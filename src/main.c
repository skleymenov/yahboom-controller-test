// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board_config.h"
#include "led.h"
#include "button.h"
#include "motor.h"
#include "encoder.h"
#include "wheel.h"
#include "lidar.h"

static const char *TAG = "MAIN";

// ── Motor wiring — 4-wheel omni chassis ──────────────────────────────────────
// pwma/pwmb swap per motor corrects physical forward direction at the wiring
// level; no sign logic needed inside motor or kinematic code.
static const motor_config_t k_motors[] = {
    { BOARD_MOTOR_M1_PWMA, BOARD_MOTOR_M1_PWMB, BOARD_MOTOR_M1_GROUP },  // left  front
    { BOARD_MOTOR_M2_PWMA, BOARD_MOTOR_M2_PWMB, BOARD_MOTOR_M2_GROUP },  // left  rear
    { BOARD_MOTOR_M3_PWMA, BOARD_MOTOR_M3_PWMB, BOARD_MOTOR_M3_GROUP },  // right front
    { BOARD_MOTOR_M4_PWMA, BOARD_MOTOR_M4_PWMB, BOARD_MOTOR_M4_GROUP },  // right rear
};

// ── Encoder wiring — chan_a/chan_b swap for M3/M4 keeps count sign consistent ─
// After the swap all four encoders count positive when the robot moves forward.
static const encoder_config_t k_encoders[] = {
    { BOARD_ENC_M1_HA, BOARD_ENC_M1_HB, BOARD_ENC_M1_HB, BOARD_ENC_M1_HA },
    { BOARD_ENC_M2_HA, BOARD_ENC_M2_HB, BOARD_ENC_M2_HB, BOARD_ENC_M2_HA },
    { BOARD_ENC_M3_HB, BOARD_ENC_M3_HA, BOARD_ENC_M3_HA, BOARD_ENC_M3_HB },
    { BOARD_ENC_M4_HB, BOARD_ENC_M4_HA, BOARD_ENC_M4_HA, BOARD_ENC_M4_HB },
};

// ── Wheel PID config — motor_id / encoder_id match the arrays above ──────────
// WHEEL_MAX_TPS: measure by spinning one free wheel at Motor_SetSpeed(200) for
// one second and reading Encoder_GetCount.  Start conservatively — a value
// that is 20% too low just means the FF slightly under-drives and the I-term
// catches up quickly.  A value that is too high over-drives and the I-term
// backs off, which is also fine but may cause brief overshoot at start.
//
// Gains: with feedforward doing the heavy lifting, keep Kp small (prevents
// overshoot), Ki moderate (trims FF error in ~0.5 s), Kd near-zero.
#define WHEEL_MAX_TPS  6000
#define WHEEL_KP       0.02f
#define WHEEL_KI       0.05f
#define WHEEL_KD       0.001f

static const wheel_config_t k_wheels[] = {
    { 0, 0, WHEEL_KP, WHEEL_KI, WHEEL_KD, WHEEL_MAX_TPS },  // left  front
    { 1, 1, WHEEL_KP, WHEEL_KI, WHEEL_KD, WHEEL_MAX_TPS },  // left  rear
    { 2, 2, WHEEL_KP, WHEEL_KI, WHEEL_KD, WHEEL_MAX_TPS },  // right front
    { 3, 3, WHEEL_KP, WHEEL_KI, WHEEL_KD, WHEEL_MAX_TPS },  // right rear
};

#define MOTOR_COUNT   (sizeof(k_motors)   / sizeof(k_motors[0]))
#define ENCODER_COUNT (sizeof(k_encoders) / sizeof(k_encoders[0]))
#define WHEEL_COUNT   (sizeof(k_wheels)   / sizeof(k_wheels[0]))

// ── Square trajectory parameters ──────────────────────────────────────────────
#define SQUARE_SIDE_TICKS  2048
// Target speed in encoder ticks/second.  Tune alongside PID gains.
#define DRIVE_SPEED_TPS    2000

// Mecanum kinematic speed vectors (indices: 0=LF, 1=LR, 2=RF, 3=RR).
// Each phase is a pure translation — the robot never rotates.
//   right strafe: LF+, LR−, RF−, RR+
//   left  strafe: LF−, LR+, RF+, RR−
static const int k_fwd[4]   = {  DRIVE_SPEED_TPS,  DRIVE_SPEED_TPS,  DRIVE_SPEED_TPS,  DRIVE_SPEED_TPS };
static const int k_right[4] = {  DRIVE_SPEED_TPS, -DRIVE_SPEED_TPS, -DRIVE_SPEED_TPS,  DRIVE_SPEED_TPS };
static const int k_bwd[4]   = { -DRIVE_SPEED_TPS, -DRIVE_SPEED_TPS, -DRIVE_SPEED_TPS, -DRIVE_SPEED_TPS };
static const int k_left[4]  = { -DRIVE_SPEED_TPS,  DRIVE_SPEED_TPS,  DRIVE_SPEED_TPS, -DRIVE_SPEED_TPS };

// ── Emergency stop flag set by BOOT button ────────────────────────────────────
static volatile bool s_stopped = false;
static TaskHandle_t  s_square_task_handle = NULL;

static button_t s_boot_btn;
static button_t s_user_btn;

static void boot_btn_cb(button_event_t event)
{
    if (event != BUTTON_EVENT_PRESSED) return;
    s_stopped = !s_stopped;
    if (s_stopped)
    {
        Wheel_StopAll();
        Led_StopBlink();
        Led_On();
        ESP_LOGI(TAG, "stopped");
    }
    else
    {
        Led_StartBlink(500);
        ESP_LOGI(TAG, "resuming");
        if (s_square_task_handle)
            xTaskNotifyGive(s_square_task_handle);
    }
}

static void user_btn_cb(button_event_t event)
{
    if (event == BUTTON_EVENT_PRESSED)
        ESP_LOGI(TAG, "speeds  %d  %d  %d  %d",
                 Wheel_GetSpeed(0), Wheel_GetSpeed(1),
                 Wheel_GetSpeed(2), Wheel_GetSpeed(3));
}

// ── Lidar logging task ────────────────────────────────────────────────────────
static void lidar_log_task(void *arg)
{
    lidar_cloud_t cloud;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        Lidar_GetCloud(&cloud);
        ESP_LOGI(TAG, "lidar  min %4u mm @ %6.1f°   max %5u mm @ %6.1f°",
                 cloud.min_dist_mm, cloud.min_dist_angle,
                 cloud.max_dist_mm, cloud.max_dist_angle);
    }
}

// ── Square trajectory task ────────────────────────────────────────────────────
// Progress is measured as the average absolute encoder delta from the phase
// start — valid for both straight and strafe phases.
static void run_phase(const int *speeds_tps, const char *label)
{
    int start[4];
    for (int i = 0; i < 4; i++)
        start[i] = Encoder_GetCount(i);

    Wheel_SetSpeedAll(speeds_tps);

    while (1)
    {
        if (s_stopped)
        {
            Wheel_StopAll();
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            // Wheels were braked during pause, so start[] is still valid.
            Wheel_SetSpeedAll(speeds_tps);
        }

        int total = 0;
        for (int i = 0; i < 4; i++)
        {
            int d = Encoder_GetCount(i) - start[i];
            total += d < 0 ? -d : d;
        }
        if (total / 4 >= SQUARE_SIDE_TICKS) break;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Wheel_StopAll();
    ESP_LOGI(TAG, "  %s done — enc %d %d %d %d", label,
             Encoder_GetCount(0), Encoder_GetCount(1),
             Encoder_GetCount(2), Encoder_GetCount(3));
    vTaskDelay(pdMS_TO_TICKS(150));
}

static void square_task(void *arg)
{
    int lap = 0;
    while (1)
    {
        ESP_LOGI(TAG, "lap %d", ++lap);
        run_phase(k_fwd,   "forward");
        run_phase(k_right, "right  ");
        run_phase(k_bwd,   "backward");
        run_phase(k_left,  "left   ");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void app_main(void)
{
    ESP_LOGI(TAG, "Starting yahboom-test");
    ESP_LOGI(TAG, "BOOT button: pause/resume  |  USER button: print speeds");

    Led_Init();
    Led_StartBlink(500);

    Button_Init(&s_boot_btn, BOARD_BTN_BOOT_GPIO, true, BUTTON_DEBOUNCE_MS, boot_btn_cb);
    Button_Init(&s_user_btn, BOARD_BTN_USER_GPIO, true, BUTTON_DEBOUNCE_MS, user_btn_cb);

    Motor_Init(k_motors, MOTOR_COUNT);
    Encoder_Init(k_encoders, ENCODER_COUNT);
    Wheel_Init(k_wheels, WHEEL_COUNT, 20);
    Lidar_Init(BOARD_LIDAR_UART_NUM, BOARD_LIDAR_TX_GPIO, BOARD_LIDAR_RX_GPIO);

    xTaskCreate(lidar_log_task, "lidar_log", 2048, NULL, 4, NULL);
    xTaskCreate(square_task,    "square",    3072, NULL, 5, &s_square_task_handle);
}
