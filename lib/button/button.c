// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

#include "button.h"

#include "driver/gpio.h"
#include "esp_err.h"

// Restart the debounce timer on every edge; the callback fires once it settles.
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    button_t *btn = (button_t *)arg;
    BaseType_t woken = pdFALSE;
    xTimerResetFromISR(btn->debounce_timer, &woken);
    portYIELD_FROM_ISR(woken);
}

// Runs in the timer daemon task after the signal has settled.
static void debounce_cb(TimerHandle_t timer)
{
    button_t *btn = (button_t *)pvTimerGetTimerID(timer);
    bool raw = gpio_get_level(btn->gpio);
    bool pressed = btn->active_low ? !raw : raw;

    if (pressed != btn->pressed)
    {
        btn->pressed = pressed;
        if (btn->callback)
        {
            btn->callback(pressed ? BUTTON_EVENT_PRESSED : BUTTON_EVENT_RELEASED);
        }
    }
}

void Button_Init(button_t *btn, gpio_num_t gpio, bool active_low,
                 uint32_t debounce_ms, button_callback_t callback)
{
    btn->gpio          = gpio;
    btn->active_low    = active_low;
    btn->pressed       = false;
    btn->callback      = callback;

    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_ANYEDGE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << gpio),
        .pull_down_en = 0,
        .pull_up_en   = active_low ? 1 : 0,
    };
    gpio_config(&io_conf);

    btn->debounce_timer = xTimerCreate("btn", pdMS_TO_TICKS(debounce_ms),
                                       pdFALSE, btn, debounce_cb);

    // Safe to call multiple times; returns ESP_ERR_INVALID_STATE if already installed.
    esp_err_t ret = gpio_install_isr_service(0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret == ESP_ERR_INVALID_STATE ? ESP_OK : ret);

    gpio_isr_handler_add(gpio, gpio_isr_handler, btn);
}
