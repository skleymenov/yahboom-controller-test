// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

#include "esp_log.h"
#include "esp_heap_caps.h"

#include "led.h"
#include "button.h"

static const char *TAG = "MAIN";

static button_t s_boot_btn;
static button_t s_user_btn;

static void boot_btn_cb(button_event_t event)
{
    if (event == BUTTON_EVENT_PRESSED)
    {
        ESP_LOGI(TAG, "BOOT pressed  — free heap: %lu bytes",
                 heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    }
}

static void user_btn_cb(button_event_t event)
{
    if (event == BUTTON_EVENT_PRESSED)
    {
        ESP_LOGI(TAG, "USER pressed  — free internal heap: %lu bytes",
                 heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting yahboom-test");

    Led_Init();
    Led_StartBlink(500);

    Button_Init(&s_boot_btn, BUTTON_GPIO_BOOT, true, BUTTON_DEBOUNCE_MS, boot_btn_cb);
    Button_Init(&s_user_btn, BUTTON_GPIO_USER, true, BUTTON_DEBOUNCE_MS, user_btn_cb);
}
