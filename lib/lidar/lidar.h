// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

// MS200 dToF Lidar driver.
//
// The lidar emits point-cloud packets at ~375 packets/s (12 points each).
// At 10 Hz frame rate it sweeps ~37 packets per revolution, so LIDAR_CLOUD_SIZE
// of 45 gives one ~8° angular bucket per slot with comfortable headroom.
//
// Each bucket is filled by the most recent packet whose angular midpoint falls
// inside it.  Fields are averaged over the 12 raw points in that packet;
// zero-distance readings (no return) are excluded from all averages.
//
// Lidar_Init() starts a background FreeRTOS task — no further calls needed
// to keep the cloud current.  All public getters are mutex-protected and safe
// to call from any task or priority.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Number of angular buckets.  360° / LIDAR_CLOUD_SIZE = degrees per bucket.
// Override before including this header if a different resolution is needed.
#ifndef LIDAR_CLOUD_SIZE
#define LIDAR_CLOUD_SIZE  45
#endif

// Data derived from one point-cloud packet (12 raw points averaged).
// `valid` is false until the lidar has swept through this angular sector
// at least once since Lidar_Init().
typedef struct {
    float    center_angle;   // midpoint of the packet's angular span (°, 0–360)
    uint16_t avg_distance;   // mean distance of valid points (mm); 0 if all invalid
    uint8_t  avg_intensity;  // mean intensity (0–255)
    uint16_t min_distance;   // closest valid point in the packet (mm)
    uint16_t max_distance;   // farthest valid point in the packet (mm)
    uint16_t speed_dps;      // rotation speed reported by lidar (°/s)
    uint16_t timestamp_ms;   // lidar timestamp (wraps at 30 000 ms)
    bool     valid;
} lidar_cloud_point_t;

// Complete point cloud snapshot.
typedef struct {
    lidar_cloud_point_t points[LIDAR_CLOUD_SIZE];
    float    min_dist_angle; // direction of closest obstacle (°)
    float    max_dist_angle; // direction of farthest obstacle (°)
    uint16_t min_dist_mm;
    uint16_t max_dist_mm;
} lidar_cloud_t;

// Start UART and background parse task.  Call once from app_main().
// uart_num: UART port index (e.g. 1 for UART_NUM_1).
// tx_gpio / rx_gpio: GPIO numbers for the lidar serial lines.
void     Lidar_Init(int uart_num, int tx_gpio, int rx_gpio);

// Copy the current cloud into *out (mutex-protected atomic snapshot).
void     Lidar_GetCloud(lidar_cloud_t *out);

// Averaged distance in the bucket closest to angle_deg (mm).
// Returns 0 if the bucket is not yet valid or all its points had no return.
uint16_t Lidar_GetDistance(float angle_deg);

// Direction angles of the globally closest / farthest obstacle.
float    Lidar_GetMinDistAngle(void);
float    Lidar_GetMaxDistAngle(void);

#ifdef __cplusplus
}
#endif
