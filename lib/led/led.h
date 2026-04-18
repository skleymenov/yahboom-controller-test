// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LED_GPIO         45
#define LED_ACTIVE_LEVEL 1  // 1 = active high

void Led_Init(void);
void Led_On(void);
void Led_Off(void);
void Led_StartBlink(uint32_t period_ms);
void Led_StopBlink(void);

#ifdef __cplusplus
}
#endif
