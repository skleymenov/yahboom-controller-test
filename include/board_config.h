// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

// Yahboom MicroROS board — pin assignments and wiring constants.
// This is the single place to update if hardware changes.

#pragma once

// ── LED ──────────────────────────────────────────────────────────────────────
#define BOARD_LED_GPIO          45
#define BOARD_LED_ACTIVE_LEVEL   1  // active high

// ── Buttons ──────────────────────────────────────────────────────────────────
#define BOARD_BTN_BOOT_GPIO      0  // BOOT button, active low
#define BOARD_BTN_USER_GPIO     42  // User button, active low

// ── Motors (M1=left-front, M2=left-rear, M3=right-front, M4=right-rear) ─────
// pwmA/pwmB assignment determines logical forward direction.
// M1 and M2 have A/B swapped relative to the PCB label because those motors
// are mounted facing the opposite direction on the chassis.
#define BOARD_MOTOR_M1_PWMA     5   // PCB label: M1B
#define BOARD_MOTOR_M1_PWMB     4   // PCB label: M1A
#define BOARD_MOTOR_M2_PWMA    16   // PCB label: M2B
#define BOARD_MOTOR_M2_PWMB    15   // PCB label: M2A
#define BOARD_MOTOR_M3_PWMA     9   // PCB label: M3A
#define BOARD_MOTOR_M3_PWMB    10   // PCB label: M3B
#define BOARD_MOTOR_M4_PWMA    13   // PCB label: M4A
#define BOARD_MOTOR_M4_PWMB    14   // PCB label: M4B

// Left side (M1/M2) on group 0, right side (M3/M4) on group 1.
// 2+2 keeps each group balanced and synchronises each lateral pair.
#define BOARD_MOTOR_M1_GROUP    0
#define BOARD_MOTOR_M2_GROUP    0
#define BOARD_MOTOR_M3_GROUP    1
#define BOARD_MOTOR_M4_GROUP    1

// ── Lidar (MS200, UART1) ──────────────────────────────────────────────────────
#define BOARD_LIDAR_UART_NUM    1   // UART_NUM_1
#define BOARD_LIDAR_TX_GPIO     17
#define BOARD_LIDAR_RX_GPIO     18

// ── Encoders ─────────────────────────────────────────────────────────────────
// M3 and M4 encoders have H-A/H-B swapped (same physical reason as motors).
#define BOARD_ENC_M1_HA         6
#define BOARD_ENC_M1_HB         7
#define BOARD_ENC_M2_HA        47
#define BOARD_ENC_M2_HB        48
#define BOARD_ENC_M3_HA        11
#define BOARD_ENC_M3_HB        12
#define BOARD_ENC_M4_HA         1
#define BOARD_ENC_M4_HB         2
