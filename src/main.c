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

#define MOTOR_COUNT   (sizeof(k_motors)   / sizeof(k_motors[0]))
#define ENCODER_COUNT (sizeof(k_encoders) / sizeof(k_encoders[0]))

// ── Square trajectory parameters ──────────────────────────────────────────────
#define SQUARE_SIDE_TICKS  2048
#define DRIVE_SPEED        (MOTOR_MAX_SPEED / 4)

// Mecanum kinematic speed vectors (indices: 0=LF, 1=LR, 2=RF, 3=RR).
// Each phase is a pure translation — the robot never rotates.
static const int k_fwd[4]   = {  DRIVE_SPEED,  DRIVE_SPEED,  DRIVE_SPEED,  DRIVE_SPEED };
static const int k_right[4] = {  DRIVE_SPEED,  0,  0,  DRIVE_SPEED };
static const int k_bwd[4]   = { -DRIVE_SPEED, -DRIVE_SPEED, -DRIVE_SPEED, -DRIVE_SPEED };
static const int k_left[4]  = { -DRIVE_SPEED,  0,  0, -DRIVE_SPEED };

// During strafing half the encoders count positive and half negative, so use
// the average of absolute values — this works for all four phases.
static int drive_ticks(void)
{
    int total = 0;
    for (int i = 0; i < 4; i++)
    {
        int c = Encoder_GetCount(i);
        total += c < 0 ? -c : c;
    }
    return total / 4;
}

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
        Motor_BrakeAll();
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
    {
        Encoder_ClearAll();
        ESP_LOGI(TAG, "encoders cleared");
    }
}

// ── Square trajectory task ────────────────────────────────────────────────────
static void run_phase(const int *speeds, const char *label)
{
    Encoder_ClearAll();
    Motor_SetSpeedAll(speeds);

    while (drive_ticks() < SQUARE_SIDE_TICKS)
    {
        if (s_stopped)
        {
            Motor_BrakeAll();
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            Motor_SetSpeedAll(speeds);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Motor_BrakeAll();
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
    ESP_LOGI(TAG, "BOOT button: pause/resume  |  USER button: clear encoders");

    Led_Init();
    Led_StartBlink(500);

    Button_Init(&s_boot_btn, BOARD_BTN_BOOT_GPIO, true, BUTTON_DEBOUNCE_MS, boot_btn_cb);
    Button_Init(&s_user_btn, BOARD_BTN_USER_GPIO, true, BUTTON_DEBOUNCE_MS, user_btn_cb);

    Motor_Init(k_motors, MOTOR_COUNT);
    Encoder_Init(k_encoders, ENCODER_COUNT);

    xTaskCreate(square_task, "square", 3072, NULL, 5, &s_square_task_handle);
}
