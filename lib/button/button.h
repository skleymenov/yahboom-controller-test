// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BUTTON_GPIO_BOOT  0   // BOOT button (active low, internal pull-up)
#define BUTTON_GPIO_USER  42  // User button (active low, internal pull-up)

#define BUTTON_DEBOUNCE_MS 20

typedef enum {
    BUTTON_EVENT_PRESSED,
    BUTTON_EVENT_RELEASED,
} button_event_t;

typedef void (*button_callback_t)(button_event_t event);

typedef struct {
    gpio_num_t       gpio;
    bool             active_low;
    bool             pressed;
    TimerHandle_t    debounce_timer;
    button_callback_t callback;
} button_t;

// active_low should be true for both board buttons (they pull GPIO to GND when pressed).
void Button_Init(button_t *btn, gpio_num_t gpio, bool active_low,
                 uint32_t debounce_ms, button_callback_t callback);

#ifdef __cplusplus
}
#endif
