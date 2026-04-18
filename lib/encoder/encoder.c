// Copyright 2026 Sergei Kleimenov
// SPDX-License-Identifier: Apache-2.0

#include "encoder.h"

#include "driver/pulse_cnt.h"
#include "esp_log.h"

static const char *TAG = "encoder";

#define PCNT_HIGH_LIMIT   1000
#define PCNT_LOW_LIMIT   -1000
#define GLITCH_FILTER_NS  1000

static pcnt_unit_handle_t s_units[ENCODER_MAX_COUNT];
static int s_count = 0;

void Encoder_Init(const encoder_config_t *configs, int count)
{
    ESP_LOGI(TAG, "initialising %d encoder(s)", count);
    s_count = count;

    pcnt_unit_config_t unit_cfg = {
        .high_limit        = PCNT_HIGH_LIMIT,
        .low_limit         = PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = GLITCH_FILTER_NS,
    };

    for (int i = 0; i < count; i++)
    {
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_units[i], &filter_cfg));

        pcnt_channel_handle_t chan_a, chan_b;

        pcnt_chan_config_t chan_a_cfg = {
            .edge_gpio_num  = configs[i].chan_a_edge_gpio,
            .level_gpio_num = configs[i].chan_a_level_gpio,
        };
        ESP_ERROR_CHECK(pcnt_new_channel(s_units[i], &chan_a_cfg, &chan_a));

        pcnt_chan_config_t chan_b_cfg = {
            .edge_gpio_num  = configs[i].chan_b_edge_gpio,
            .level_gpio_num = configs[i].chan_b_level_gpio,
        };
        ESP_ERROR_CHECK(pcnt_new_channel(s_units[i], &chan_b_cfg, &chan_b));

        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_units[i], PCNT_HIGH_LIMIT));
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_units[i], PCNT_LOW_LIMIT));
        ESP_ERROR_CHECK(pcnt_unit_enable(s_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(s_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_start(s_units[i]));
    }
}

int Encoder_GetCount(int id)
{
    if (id < 0 || id >= s_count) return 0;
    int count = 0;
    pcnt_unit_get_count(s_units[id], &count);
    return count;
}

void Encoder_ClearCount(int id)
{
    if (id < 0 || id >= s_count) return;
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_units[id]));
}

void Encoder_ClearAll(void)
{
    for (int i = 0; i < s_count; i++)
        ESP_ERROR_CHECK(pcnt_unit_clear_count(s_units[i]));
}
