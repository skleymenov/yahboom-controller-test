// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

#include "motor.h"

#include "bdc_motor.h"
#include "esp_log.h"

static const char *TAG = "motor";

static bdc_motor_handle_t s_motors[MOTOR_MAX_COUNT];
static int s_count = 0;

static int clamp_speed(int speed)
{
    if (speed >  MOTOR_DUTY_TICK_MAX) return  MOTOR_DUTY_TICK_MAX;
    if (speed < -MOTOR_DUTY_TICK_MAX) return -MOTOR_DUTY_TICK_MAX;
    return speed;
}

static int apply_dead_zone(int speed)
{
    if (speed > 0) return speed + MOTOR_DEAD_ZONE;
    if (speed < 0) return speed - MOTOR_DEAD_ZONE;
    return 0;
}

void Motor_Init(const motor_config_t *configs, int count)
{
    ESP_LOGI(TAG, "initialising %d motor(s)", count);
    s_count = count;

    for (int i = 0; i < count; i++)
    {
        bdc_motor_config_t motor_cfg = {
            .pwm_freq_hz   = MOTOR_PWM_FREQ_HZ,
            .pwma_gpio_num = configs[i].pwma_gpio,
            .pwmb_gpio_num = configs[i].pwmb_gpio,
        };
        bdc_motor_mcpwm_config_t mcpwm_cfg = {
            .group_id      = configs[i].mcpwm_group,
            .resolution_hz = MOTOR_TIMER_RESOLUTION_HZ,
        };
        ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_cfg, &mcpwm_cfg, &s_motors[i]));
        ESP_ERROR_CHECK(bdc_motor_enable(s_motors[i]));
    }
}

void Motor_SetSpeed(int id, int speed)
{
    if (id < 0 || id >= s_count) return;

    speed = clamp_speed(apply_dead_zone(speed));

    if (speed > 0)
    {
        ESP_ERROR_CHECK(bdc_motor_forward(s_motors[id]));
        ESP_ERROR_CHECK(bdc_motor_set_speed(s_motors[id], speed));
    }
    else if (speed < 0)
    {
        ESP_ERROR_CHECK(bdc_motor_reverse(s_motors[id]));
        ESP_ERROR_CHECK(bdc_motor_set_speed(s_motors[id], -speed));
    }
    else
    {
        ESP_ERROR_CHECK(bdc_motor_coast(s_motors[id]));
    }
}

void Motor_SetSpeedAll(const int *speeds)
{
    for (int i = 0; i < s_count; i++)
        Motor_SetSpeed(i, speeds[i]);
}

void Motor_Brake(int id)
{
    if (id < 0 || id >= s_count) return;
    ESP_ERROR_CHECK(bdc_motor_brake(s_motors[id]));
}

void Motor_BrakeAll(void)
{
    for (int i = 0; i < s_count; i++)
        ESP_ERROR_CHECK(bdc_motor_brake(s_motors[i]));
}

void Motor_Coast(int id)
{
    if (id < 0 || id >= s_count) return;
    ESP_ERROR_CHECK(bdc_motor_coast(s_motors[id]));
}

void Motor_CoastAll(void)
{
    for (int i = 0; i < s_count; i++)
        ESP_ERROR_CHECK(bdc_motor_coast(s_motors[i]));
}
