// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

#include "unity.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "button.h"

static button_t s_boot_btn;
static button_t s_user_btn;
static volatile bool s_cb_fired = false;

static void test_cb(button_event_t event)
{
    s_cb_fired = true;
}

void setUp(void)
{
    s_cb_fired = false;
}

void tearDown(void) {}

// With pull-up enabled and no button pressed, GPIO reads high.
void test_boot_button_reads_high_when_released(void)
{
    TEST_ASSERT_EQUAL_INT(1, gpio_get_level(BUTTON_GPIO_BOOT));
}

void test_user_button_reads_high_when_released(void)
{
    TEST_ASSERT_EQUAL_INT(1, gpio_get_level(BUTTON_GPIO_USER));
}

// No edge occurs at init, so the debounce timer never fires and the callback is not called.
void test_no_spurious_callback_on_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_FALSE(s_cb_fired);
}

void app_main(void)
{
    Button_Init(&s_boot_btn, BUTTON_GPIO_BOOT, true, BUTTON_DEBOUNCE_MS, test_cb);
    Button_Init(&s_user_btn, BUTTON_GPIO_USER, true, BUTTON_DEBOUNCE_MS, test_cb);

    UNITY_BEGIN();
    RUN_TEST(test_boot_button_reads_high_when_released);
    RUN_TEST(test_user_button_reads_high_when_released);
    RUN_TEST(test_no_spurious_callback_on_init);
    UNITY_END();
}
