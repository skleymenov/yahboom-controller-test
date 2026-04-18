// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

#include "led.h"

#include <stdbool.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

static TimerHandle_t s_blink_timer = NULL;

static void blink_cb(TimerHandle_t timer)
{
    static bool on = false;
    on = !on;
    on ? Led_On() : Led_Off();
}

void Led_Init(void)
{
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_GPIO),
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    gpio_config(&io_conf);
    Led_Off();
}

void Led_On(void)
{
    gpio_set_level(LED_GPIO, LED_ACTIVE_LEVEL);
}

void Led_Off(void)
{
    gpio_set_level(LED_GPIO, LED_ACTIVE_LEVEL ^ 1);
}

void Led_StartBlink(uint32_t period_ms)
{
    if (s_blink_timer == NULL)
    {
        s_blink_timer = xTimerCreate("led", pdMS_TO_TICKS(period_ms),
                                     pdTRUE, NULL, blink_cb);
    }
    xTimerStart(s_blink_timer, 0);
}

void Led_StopBlink(void)
{
    if (s_blink_timer != NULL)
    {
        xTimerStop(s_blink_timer, 0);
        Led_Off();
    }
}
