// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

#include "unity.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led.h"

void setUp(void)
{
    Led_StopBlink();
    Led_Off();
}

void tearDown(void)
{
    Led_StopBlink();
    Led_Off();
}

void test_led_on(void)
{
    Led_On();
    TEST_ASSERT_EQUAL_INT(LED_ACTIVE_LEVEL, gpio_get_level(LED_GPIO));
}

void test_led_off(void)
{
    Led_On();
    Led_Off();
    TEST_ASSERT_EQUAL_INT(LED_ACTIVE_LEVEL ^ 1, gpio_get_level(LED_GPIO));
}

// LED starts off; after one 200 ms period it should be on, after two it should be off.
void test_led_blink_toggles(void)
{
    Led_StartBlink(200);
    vTaskDelay(pdMS_TO_TICKS(300));
    TEST_ASSERT_EQUAL_INT(LED_ACTIVE_LEVEL, gpio_get_level(LED_GPIO));
    vTaskDelay(pdMS_TO_TICKS(200));
    TEST_ASSERT_EQUAL_INT(LED_ACTIVE_LEVEL ^ 1, gpio_get_level(LED_GPIO));
}

void test_led_stop_turns_off(void)
{
    Led_StartBlink(200);
    vTaskDelay(pdMS_TO_TICKS(300));
    Led_StopBlink();
    TEST_ASSERT_EQUAL_INT(LED_ACTIVE_LEVEL ^ 1, gpio_get_level(LED_GPIO));
}

void app_main(void)
{
    Led_Init();

    UNITY_BEGIN();
    RUN_TEST(test_led_on);
    RUN_TEST(test_led_off);
    RUN_TEST(test_led_blink_toggles);
    RUN_TEST(test_led_stop_turns_off);
    UNITY_END();
}
