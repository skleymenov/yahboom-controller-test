// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

#include "esp_log.h"

#include "led.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting yahboom-test");
    Led_Init();
    Led_StartBlink(500);
}
