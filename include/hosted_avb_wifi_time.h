/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Wi-Fi time on the host (CONFIG_HOSTED_AVB_WIFI_TIME): receives the
 * measurements a coprocessor station's FTM time engine makes and puts
 * each on the host's PTP clock through the clock link. Every 10 s it logs
 * how far the Wi-Fi time sat from the host clock (WIFITIME lines), which
 * is the transfer's error while the host is synchronized by wire.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One measurement on the host's clock. */
typedef struct {
  int64_t host_ns;      /* host PTP clock at the measurement */
  int64_t reference_ns; /* gPTP time there, over Wi-Fi */
  int64_t received_us;  /* esp_timer_get_time() on arrival */
  int32_t rtt_ns;
  uint16_t time_base;
  uint8_t source_port[10];
  uint8_t domain;
} hosted_avb_wifi_time_sample_t;

/* Starts receiving; needs hosted_avb_clock_link_start(). */
esp_err_t hosted_avb_wifi_time_start(void);

/* The latest measurement; false before the first. */
bool hosted_avb_wifi_time_latest(hosted_avb_wifi_time_sample_t *sample);

/* Called with every measurement as it is put on the host clock, on the
 * receiver's task; keep it short. NULL to stop. */
typedef void (*hosted_avb_wifi_time_callback_t)(
    const hosted_avb_wifi_time_sample_t *sample, void *context);
void hosted_avb_wifi_time_set_callback(hosted_avb_wifi_time_callback_t callback,
                                       void *context);

#ifdef __cplusplus
}
#endif
