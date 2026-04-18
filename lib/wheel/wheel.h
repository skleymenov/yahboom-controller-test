// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WHEEL_MAX_COUNT  6

// Per-wheel wiring and PID tuning, supplied by the caller.
// motor_id and encoder_id are 0-based indices into the arrays passed to
// Motor_Init() and Encoder_Init() respectively.
//
// max_tps: empirical top speed (ticks/s) at MOTOR_MAX_SPEED — enables a
// feedforward term that puts the motor near the right duty immediately, so
// the PID only needs to trim the residual error.  Measure it by running
// Motor_SetSpeed(MOTOR_MAX_SPEED) on a free wheel and reading Encoder_GetCount
// over one second.  Set to 0 to disable feedforward.
typedef struct {
    int   motor_id;
    int   encoder_id;
    float kp;
    float ki;
    float kd;
    int   max_tps;
} wheel_config_t;

// Initialize the wheel layer.  Motor_Init() and Encoder_Init() must have been
// called first.  period_ms is the PID update interval (10–50 ms is typical).
void Wheel_Init(const wheel_config_t *configs, int count, int period_ms);

// Set commanded speed in encoder ticks/second.  Negative = reverse.
// Resets the PID state so each new command starts fresh.
void Wheel_SetSpeed(int id, int speed_tps);
void Wheel_SetSpeedAll(const int *speeds_tps);

// Immediately brake and clear PID state.
void Wheel_Stop(int id);
void Wheel_StopAll(void);

// Most recently measured speed in ticks/second.
int  Wheel_GetSpeed(int id);

#ifdef __cplusplus
}
#endif
