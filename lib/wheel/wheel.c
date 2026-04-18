// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

#include "wheel.h"
#include "motor.h"
#include "encoder.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "wheel";

// Anti-windup: cap the accumulated integral in ticks (error_tps × dt).
// With Ki=0.05 and limit=600: max I-term contribution = 0.05 × 600 = 30 motor
// units — enough to trim feedforward error without dominating the output.
#define INTEGRAL_LIMIT  600.0f

typedef struct {
    int   motor_id;
    int   encoder_id;
    float kp, ki, kd;
    int   max_tps;

    volatile int target_tps;   // commanded ticks/s
    int          prev_count;   // encoder count at the last PID tick
    float        integral;
    int          prev_error;
    bool         first_tick;   // suppress derivative spike after setpoint change
    volatile int measured_tps; // speed measured at the last PID tick
} wheel_state_t;

static wheel_state_t s_wheels[WHEEL_MAX_COUNT];
static int           s_count     = 0;
static int           s_period_ms = 20;

static void pid_task(void *arg)
{
    const float dt        = s_period_ms / 1000.0f;
    TickType_t  last_wake = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(s_period_ms));

        for (int i = 0; i < s_count; i++)
        {
            wheel_state_t *w      = &s_wheels[i];
            int            target = w->target_tps;

            // Measure speed from encoder delta since last tick.
            int count       = Encoder_GetCount(w->encoder_id);
            int delta       = count - w->prev_count;
            w->prev_count   = count;
            w->measured_tps = (int)(delta / dt);

            if (target == 0)
            {
                Motor_Brake(w->motor_id);
                w->integral   = 0.0f;
                w->prev_error = 0;
                w->first_tick = true;
                continue;
            }

            // Feedforward: map target speed linearly to the motor command range.
            // This puts the wheel near the right duty immediately; PID trims the
            // residual.  If max_tps is zero, feedforward is disabled.
            int ff = 0;
            if (w->max_tps > 0)
            {
                ff = (int)((long)target * MOTOR_MAX_SPEED / w->max_tps);
                if      (ff >  MOTOR_MAX_SPEED) ff =  MOTOR_MAX_SPEED;
                else if (ff < -MOTOR_MAX_SPEED) ff = -MOTOR_MAX_SPEED;
            }

            int error = target - w->measured_tps;

            // On the first tick after a setpoint change, skip the derivative so
            // the (error - 0) / dt impulse does not spike the output.
            float deriv = 0.0f;
            if (!w->first_tick)
                deriv = (float)(error - w->prev_error) / dt;
            w->first_tick = false;

            w->integral += error * dt;
            if      (w->integral >  INTEGRAL_LIMIT) w->integral =  INTEGRAL_LIMIT;
            else if (w->integral < -INTEGRAL_LIMIT) w->integral = -INTEGRAL_LIMIT;

            w->prev_error = error;

            int correction = (int)(w->kp * error + w->ki * w->integral + w->kd * deriv);
            int output     = ff + correction;

            if      (output >  MOTOR_MAX_SPEED) output =  MOTOR_MAX_SPEED;
            else if (output < -MOTOR_MAX_SPEED) output = -MOTOR_MAX_SPEED;

            Motor_SetSpeed(w->motor_id, output);
        }
    }
}

void Wheel_Init(const wheel_config_t *configs, int count, int period_ms)
{
    ESP_LOGI(TAG, "initialising %d wheel(s), period %d ms", count, period_ms);
    s_count     = count;
    s_period_ms = period_ms;

    for (int i = 0; i < count; i++)
    {
        s_wheels[i] = (wheel_state_t){
            .motor_id   = configs[i].motor_id,
            .encoder_id = configs[i].encoder_id,
            .kp         = configs[i].kp,
            .ki         = configs[i].ki,
            .kd         = configs[i].kd,
            .max_tps    = configs[i].max_tps,
            .first_tick = true,
        };
    }

    // Priority above the application task so PID updates are never starved.
    xTaskCreate(pid_task, "wheel_pid", 3072, NULL, 6, NULL);
}

void Wheel_SetSpeed(int id, int speed_tps)
{
    if (id < 0 || id >= s_count) return;
    wheel_state_t *w = &s_wheels[id];
    w->integral    = 0.0f;
    w->prev_error  = 0;
    w->first_tick  = true;
    w->prev_count  = Encoder_GetCount(w->encoder_id);
    w->target_tps  = speed_tps;   // written last — PID task reads this
}

void Wheel_SetSpeedAll(const int *speeds_tps)
{
    for (int i = 0; i < s_count; i++)
        Wheel_SetSpeed(i, speeds_tps[i]);
}

void Wheel_Stop(int id)
{
    if (id < 0 || id >= s_count) return;
    s_wheels[id].target_tps = 0;
    Motor_Brake(s_wheels[id].motor_id);
    s_wheels[id].integral   = 0.0f;
    s_wheels[id].prev_error = 0;
    s_wheels[id].first_tick = true;
}

void Wheel_StopAll(void)
{
    for (int i = 0; i < s_count; i++)
        Wheel_Stop(i);
}

int Wheel_GetSpeed(int id)
{
    if (id < 0 || id >= s_count) return 0;
    return s_wheels[id].measured_tps;
}
