// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Timer: 10 MHz resolution, 25 kHz PWM → 400 ticks full scale.
// Dead zone shifts the usable range to [200, 400] to overcome the 310 motor's
// minimum starting voltage (~3 V). User-facing speed is ±MOTOR_MAX_SPEED.
#define MOTOR_TIMER_RESOLUTION_HZ  10000000
#define MOTOR_PWM_FREQ_HZ          25000
#define MOTOR_DUTY_TICK_MAX        (MOTOR_TIMER_RESOLUTION_HZ / MOTOR_PWM_FREQ_HZ)  // 400
#define MOTOR_DEAD_ZONE            100
#define MOTOR_MAX_SPEED            (MOTOR_DUTY_TICK_MAX - MOTOR_DEAD_ZONE)           // 200

// ESP32-S3 has 2 MCPWM groups × 3 operators each.
#define MOTOR_MAX_COUNT  6

// Per-motor wiring configuration supplied by the caller.
// Swap pwma_gpio/pwmb_gpio to reverse the logical forward direction for a
// given motor without touching the library.
// mcpwm_group: 0 or 1. Each group supports up to 3 motors.
typedef struct {
    int pwma_gpio;
    int pwmb_gpio;
    int mcpwm_group;
} motor_config_t;

// Initialize motors from a caller-supplied config array.
// configs and count must remain valid for the lifetime of the application.
void Motor_Init(const motor_config_t *configs, int count);

// Set speed for one motor. id: 0-based index into the configs array.
// speed: -MOTOR_MAX_SPEED (full reverse) to +MOTOR_MAX_SPEED (full forward).
// speed 0 coasts.
void Motor_SetSpeed(int id, int speed);

// Set speed for all motors. speeds[] must have the same length as count
// passed to Motor_Init.
void Motor_SetSpeedAll(const int *speeds);

// Stop modes: brake shorts the motor terminals (resists movement);
// coast lets the motor spin down freely under inertia.
void Motor_Brake(int id);
void Motor_BrakeAll(void);
void Motor_Coast(int id);
void Motor_CoastAll(void);

#ifdef __cplusplus
}
#endif
