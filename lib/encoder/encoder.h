// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 13-line Hall encoder × 4 edges × 20:1 gear ratio = 1040 pulses per wheel revolution.
#define ENCODER_PULSES_PER_REV  1040

// ESP32-S3 has 4 PCNT units.
#define ENCODER_MAX_COUNT  4

// Per-encoder wiring configuration supplied by the caller.
// Swap chan_a / chan_b GPIOs to reverse the count sign for a given encoder
// without touching the library.
typedef struct {
    int chan_a_edge_gpio;
    int chan_a_level_gpio;
    int chan_b_edge_gpio;
    int chan_b_level_gpio;
} encoder_config_t;

// Initialize encoders from a caller-supplied config array.
void Encoder_Init(const encoder_config_t *configs, int count);

// Return accumulated pulse count. Positive = forward, negative = reverse.
// id: 0-based index into the configs array.
int  Encoder_GetCount(int id);

void Encoder_ClearCount(int id);
void Encoder_ClearAll(void);

#ifdef __cplusplus
}
#endif
