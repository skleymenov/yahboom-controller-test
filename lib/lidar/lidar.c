// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

#include "lidar.h"

#include <math.h>
#include <string.h>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "lidar";

// ── Protocol constants ────────────────────────────────────────────────────────
#define PKT_HEAD        0x54u   // point-cloud frame header
#define SYS_HEAD_1      0xAAu   // system message header byte 1
#define SYS_HEAD_2      0x55u   // system message header byte 2
#define SYS_TAIL_1      0x31u
#define SYS_TAIL_2      0xF2u
#define FLAG_SN         0x01u
#define FLAG_VERSION    0x02u
#define POINTS_PER_PKT  12      // fixed by the MS200 protocol

// Maximum packet sizes:
//   point cloud : POINTS_PER_PKT*3 + 11 = 47 bytes
//   system msg  : up to ~60 bytes in practice; 64 is safe
#define RX_BUF_SIZE     64

// ── CRC-8 look-up table (poly 0x4D: x^6 + x^3 + x^2 + 1) ───────────────────
static const uint8_t CRC_TABLE[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8,
};

static uint8_t crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++)
        crc = CRC_TABLE[(crc ^ data[i]) & 0xFF];
    return crc;
}

// ── Internal raw packet ───────────────────────────────────────────────────────
typedef struct {
    uint8_t  count;
    uint16_t speed;         // °/s
    uint16_t start_angle;   // 0.01° units
    uint16_t end_angle;     // 0.01° units
    uint16_t timestamp;     // ms
    struct { uint16_t distance; uint8_t intensity; } points[POINTS_PER_PKT];
} raw_packet_t;

// ── Shared cloud state ────────────────────────────────────────────────────────
static SemaphoreHandle_t s_mutex;
static lidar_cloud_t     s_cloud;
static int               s_uart_num;
// min_dist_mm == 0 serves as "no valid data yet" sentinel (valid distances ≥ 80 mm).

// ── Cloud helpers (always called with s_mutex held) ───────────────────────────

// Full O(N) rescan — called only when the previous extremum bucket degrades.
static void rescan_minmax(void)
{
    uint16_t gmin = UINT16_MAX, gmax = 0;
    int      min_b = -1,        max_b = -1;

    for (int i = 0; i < LIDAR_CLOUD_SIZE; i++)
    {
        const lidar_cloud_point_t *p = &s_cloud.points[i];
        if (!p->valid || p->avg_distance == 0) continue;
        if (p->avg_distance < gmin) { gmin = p->avg_distance; min_b = i; }
        if (p->avg_distance > gmax) { gmax = p->avg_distance; max_b = i; }
    }
    // Reset to 0 if no valid data remains.
    s_cloud.min_dist_mm    = (min_b >= 0) ? gmin : 0;
    s_cloud.max_dist_mm    = (max_b >= 0) ? gmax : 0;
    if (min_b >= 0) s_cloud.min_dist_angle = s_cloud.points[min_b].center_angle;
    if (max_b >= 0) s_cloud.max_dist_angle = s_cloud.points[max_b].center_angle;
}

// O(1) update.  old_d is the bucket's avg_distance *before* it was overwritten;
// new_d and center are the freshly written values.
// A full rescan is triggered only when the old value equalled a global extremum
// and the new value is worse — an infrequent event in normal operation.
static void update_minmax(uint16_t new_d, uint16_t old_d, float center)
{
    bool need_rescan = false;

    // ── Minimum ──────────────────────────────────────────────────────────────
    if (new_d > 0)
    {
        if (s_cloud.min_dist_mm == 0 || new_d <= s_cloud.min_dist_mm)
        {
            s_cloud.min_dist_mm    = new_d;
            s_cloud.min_dist_angle = center;
        }
        else if (old_d > 0 && old_d == s_cloud.min_dist_mm)
            need_rescan = true;   // old value was the global min, new is worse
    }
    else if (old_d > 0 && old_d == s_cloud.min_dist_mm)
        need_rescan = true;       // bucket went invalid, was the global min

    // ── Maximum ──────────────────────────────────────────────────────────────
    if (new_d > 0)
    {
        if (new_d >= s_cloud.max_dist_mm)
        {
            s_cloud.max_dist_mm    = new_d;
            s_cloud.max_dist_angle = center;
        }
        else if (old_d > 0 && old_d == s_cloud.max_dist_mm)
            need_rescan = true;   // old value was the global max, new is worse
    }
    else if (old_d > 0 && old_d == s_cloud.max_dist_mm)
        need_rescan = true;       // bucket went invalid, was the global max

    if (need_rescan)
        rescan_minmax();
}

static void commit_packet(const raw_packet_t *pkt)
{
    // Per-packet stats — exclude zero-distance (no-return) points.
    uint32_t dist_sum = 0, int_sum = 0;
    uint16_t min_d = UINT16_MAX, max_d = 0;
    int n = 0;

    for (int i = 0; i < pkt->count && i < POINTS_PER_PKT; i++)
    {
        uint16_t d = pkt->points[i].distance;
        if (d == 0) continue;
        dist_sum += d;
        int_sum  += pkt->points[i].intensity;
        if (d < min_d) min_d = d;
        if (d > max_d) max_d = d;
        n++;
    }

    // Center angle, handling the 360°→0° wrap.
    float start  = pkt->start_angle / 100.0f;
    float end    = pkt->end_angle   / 100.0f;
    if (end < start) end += 360.0f;
    float center = fmodf((start + end) * 0.5f, 360.0f);

    int bucket = (int)(center * LIDAR_CLOUD_SIZE / 360.0f);
    if (bucket < 0)                  bucket = 0;
    if (bucket >= LIDAR_CLOUD_SIZE)  bucket = LIDAR_CLOUD_SIZE - 1;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    lidar_cloud_point_t *slot = &s_cloud.points[bucket];
    uint16_t old_d      = slot->avg_distance;   // capture before overwrite
    slot->center_angle  = center;
    slot->avg_distance  = n ? (uint16_t)(dist_sum / (uint32_t)n) : 0;
    slot->avg_intensity = n ? (uint8_t) (int_sum  / (uint32_t)n) : 0;
    slot->min_distance  = n ? min_d : 0;
    slot->max_distance  = n ? max_d : 0;
    slot->speed_dps     = pkt->speed;
    slot->timestamp_ms  = pkt->timestamp;
    slot->valid         = true;
    update_minmax(slot->avg_distance, old_d, center);

    xSemaphoreGive(s_mutex);
}

// ── Byte-level protocol state machine ─────────────────────────────────────────
//
// MS200 emits two frame types:
//   Point cloud : 0x54, count, speed(2), start(2), N×point(3), end(2), ts(2), crc
//   System msg  : 0xAA 0x55, flag, len, data[len], crc, 0x31 0xF2
//
// All multi-byte integers are little-endian.
typedef enum
{
    ST_IDLE = 0,
    ST_PKT_COUNT,   // received 0x54, awaiting count byte
    ST_PKT_BODY,    // accumulating remaining packet bytes
    ST_SYS_HEAD2,   // received 0xAA, awaiting 0x55
    ST_SYS_FLAG,    // awaiting flag byte
    ST_SYS_LEN,     // awaiting data-length byte
    ST_SYS_BODY,    // accumulating system message body
} parse_state_t;

static void parse_byte(uint8_t b)
{
    static parse_state_t state      = ST_IDLE;
    static uint8_t       buf[RX_BUF_SIZE];
    static uint8_t       idx        = 0;
    static uint8_t       target_len = 0;

    switch (state)
    {
    case ST_IDLE:
        if (b == PKT_HEAD)
        {
            buf[0] = b;
            idx    = 1;
            state  = ST_PKT_COUNT;
        }
        else if (b == SYS_HEAD_1)
        {
            buf[0] = b;
            idx    = 1;
            state  = ST_SYS_HEAD2;
        }
        break;

    // ── Point-cloud path ──────────────────────────────────────────────────────
    case ST_PKT_COUNT:
        buf[idx++] = b;
        target_len = (b & 0x1Fu) * 3u + 11u;   // total frame length in bytes
        if (target_len > RX_BUF_SIZE)
            { state = ST_IDLE; idx = 0; break; }
        state = ST_PKT_BODY;
        break;

    case ST_PKT_BODY:
        if (idx < RX_BUF_SIZE) buf[idx] = b;
        idx++;
        if (idx < target_len) break;

        // Full frame received — validate CRC, then extract fields.
        if (crc8(buf, target_len - 1) == buf[target_len - 1])
        {
            raw_packet_t pkt;
            uint8_t n   = buf[1] & 0x1Fu;
            pkt.count       = n;
            pkt.speed       = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
            pkt.start_angle = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
            // end_angle at [target_len-5..target_len-4], timestamp at [-3..-2]
            pkt.end_angle   = (uint16_t)buf[target_len - 5] | ((uint16_t)buf[target_len - 4] << 8);
            pkt.timestamp   = (uint16_t)buf[target_len - 3] | ((uint16_t)buf[target_len - 2] << 8);
            for (int i = 0; i < (int)n && i < POINTS_PER_PKT; i++)
            {
                pkt.points[i].distance  = (uint16_t)buf[6 + 3*i] | ((uint16_t)buf[6 + 3*i + 1] << 8);
                pkt.points[i].intensity = buf[6 + 3*i + 2];
            }
            commit_packet(&pkt);
        }
        state = ST_IDLE;
        idx   = 0;
        break;

    // ── System-message path ───────────────────────────────────────────────────
    case ST_SYS_HEAD2:
        if (b == SYS_HEAD_2) { buf[idx++] = b; state = ST_SYS_FLAG; }
        else                 { state = ST_IDLE; idx = 0; }
        break;

    case ST_SYS_FLAG:
        buf[idx++] = b;
        state = ST_SYS_LEN;
        break;

    case ST_SYS_LEN:
    {
        buf[idx++] = b;
        // Frame: header(4) + data(len) + crc(1) + tail(2) = len+7 bytes total.
        uint8_t tl = 4u + b + 3u;
        if (tl > RX_BUF_SIZE) { state = ST_IDLE; idx = 0; break; }
        target_len = tl;
        state = ST_SYS_BODY;
        break;
    }

    case ST_SYS_BODY:
        if (idx < RX_BUF_SIZE) buf[idx] = b;
        idx++;
        if (idx < target_len) break;

        // Verify tail then CRC.
        if (buf[target_len - 2] == SYS_TAIL_1 && buf[target_len - 1] == SYS_TAIL_2)
        {
            uint8_t data_len = buf[3];
            if (crc8(buf, data_len + 4u) == buf[data_len + 4u])
            {
                if (buf[2] == FLAG_SN)
                    ESP_LOGI(TAG, "SN: %.*s", (int)data_len, (char *)&buf[4]);
                else if (buf[2] == FLAG_VERSION)
                    ESP_LOGI(TAG, "firmware: %.*s", (int)(data_len - 1), (char *)&buf[5]);
            }
        }
        state = ST_IDLE;
        idx   = 0;
        break;

    default:
        state = ST_IDLE;
        idx   = 0;
        break;
    }
}

// ── UART receive / parse task ─────────────────────────────────────────────────
static void lidar_task(void *arg)
{
    uint8_t rx[128];
    while (1)
    {
        int n = uart_read_bytes(s_uart_num, rx, sizeof(rx), pdMS_TO_TICKS(5));
        for (int i = 0; i < n; i++)
            parse_byte(rx[i]);
    }
}

// ── Public API ────────────────────────────────────────────────────────────────
void Lidar_Init(int uart_num, int tx_gpio, int rx_gpio)
{
    ESP_LOGI(TAG, "initialising MS200 on UART%d TX=%d RX=%d", uart_num, tx_gpio, rx_gpio);

    s_uart_num = uart_num;
    s_mutex    = xSemaphoreCreateMutex();
    memset(&s_cloud, 0, sizeof(s_cloud));

    const uart_config_t uart_cfg = {
        .baud_rate  = 230400,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_gpio, rx_gpio,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(lidar_task, "lidar", 4096, NULL, 10, NULL);
}

void Lidar_GetCloud(lidar_cloud_t *out)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_cloud;
    xSemaphoreGive(s_mutex);
}

uint16_t Lidar_GetDistance(float angle_deg)
{
    while (angle_deg <    0.0f) angle_deg += 360.0f;
    while (angle_deg >= 360.0f) angle_deg -= 360.0f;
    int bucket = (int)(angle_deg * LIDAR_CLOUD_SIZE / 360.0f);
    if (bucket < 0)                  bucket = 0;
    if (bucket >= LIDAR_CLOUD_SIZE)  bucket = LIDAR_CLOUD_SIZE - 1;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint16_t d = s_cloud.points[bucket].avg_distance;
    xSemaphoreGive(s_mutex);
    return d;
}

float Lidar_GetMinDistAngle(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    float a = s_cloud.min_dist_angle;
    xSemaphoreGive(s_mutex);
    return a;
}

float Lidar_GetMaxDistAngle(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    float a = s_cloud.max_dist_angle;
    xSemaphoreGive(s_mutex);
    return a;
}
