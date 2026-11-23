/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Clock link (CONFIG_HOSTED_AVB_CLOCK_LINK): a wire from a host GPIO to a
 * coprocessor GPIO, R52 on the Waveshare ESP32-P4-WIFI6-POE-ETH (P4 GPIO6
 * to C6 GPIO2). The host pulses it; both chips latch the edge on a 40 MHz
 * timer and bracket a read of their clock with that timer, and the
 * coprocessor sends its latch back. Each edge then pairs the host's PTP
 * clock with the coprocessor's timer and Wi-Fi MAC clock, so time can
 * move between the chips at well under a microsecond.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One edge seen by both chips. */
typedef struct {
  uint32_t sequence;
  int64_t host_ptp_ns;    /* host PTP clock at the edge */
  uint64_t cp_ticks;      /* coprocessor 40 MHz timer at the edge */
  uint64_t cp_mac_ticks;  /* the timer at the middle of the MAC read */
  uint32_t cp_mac_us;     /* coprocessor Wi-Fi MAC clock then */
} hosted_avb_clock_pair_t;

/* Host: starts pulsing the link every CONFIG_HOSTED_AVB_CLOCK_LINK_PERIOD_MS
 * and pairing the edges; needs the host's PTP clock running. */
esp_err_t hosted_avb_clock_link_start(void);

/* Host: the latest edge both chips saw; false before the first. */
bool hosted_avb_clock_link_latest(hosted_avb_clock_pair_t *pair);

/* Host: the host PTP clock at a coprocessor timer time (its ticks in ns,
 * as hosted_avb_clock_link_coprocessor_read gives them), from a fit over
 * the last edges. False until enough edges have paired, while they are
 * stale, or for a time far from them. */
bool hosted_avb_clock_link_to_host(int64_t cp_ns, int64_t *host_ns);

/* Host: the fit itself, for the coprocessor: at coprocessor time cp_ns
 * (the latest edge) the host clock read host_ns, and it runs rate host
 * ns per coprocessor ns, for the coprocessor's boot coprocessor_boot.
 * False when hosted_avb_clock_link_to_host would be. */
bool hosted_avb_clock_link_fit(int64_t *cp_ns, int64_t *host_ns,
                               double *rate, uint32_t *coprocessor_boot);

/* Coprocessor: arms the GPIO capture and answers the host (called from
 * hosted_avb_init). */
esp_err_t hosted_avb_clock_link_coprocessor_init(void);

/* Coprocessor: this boot's random ID, which the host's fit carries. */
uint32_t hosted_avb_clock_link_coprocessor_boot(void);

/* Coprocessor: reads a 32-bit register between two latches of the link's
 * timer, in that timer's ns (the domain the host can convert). False
 * before the link has started. Callable from a task. */
bool hosted_avb_clock_link_coprocessor_read(volatile const uint32_t *reg,
                                            int64_t *before_ns, uint32_t *value,
                                            int64_t *after_ns);

/* Coprocessor: the Wi-Fi MAC clock on the link's timer. At MAC time
 * mac_us exactly, the timer read cp_ns (within uncertainty_ns). */
typedef struct {
  bool valid;
  uint32_t mac_us;
  int64_t cp_ns;
  uint32_t uncertainty_ns;
} hosted_avb_mac_map_t;

/* Coprocessor (ESP32-C6): maps the MAC clock from 256 bracketed reads at
 * random phases against its microsecond steps, about 0.5 ms of work.
 * False before the link has started or when the reads disagree. */
bool hosted_avb_clock_link_map_mac(hosted_avb_mac_map_t *map);

/* Coprocessor (ESP32-C6): the access point's TSF on the link's timer,
 * read between two latches of the timer, the tightest of a few tries:
 * when the TSF reached tsf_us the timer read cp_ns, within
 * uncertainty_ns (the bracket and the TSF's microsecond step). TSF and
 * timer run from the same crystal. False before the link has started or
 * without an access point. Callable from a task. */
bool hosted_avb_clock_link_ap_tsf(int64_t *tsf_us, int64_t *cp_ns,
                                  uint32_t *uncertainty_ns);

/* A MAC-clock timestamp in ps (an FTM t1..t4 on this chip) on the link's
 * timer, within a second of the map. */
static inline bool hosted_avb_mac_map_convert(const hosted_avb_mac_map_t *map,
                                              uint64_t mac_ps, int64_t *cp_ns) {
  int64_t delta_us = (int32_t)((uint32_t)(mac_ps / 1000000) - map->mac_us);
  if (!map->valid || delta_us < -1000000 || delta_us > 1000000)
    return false;
  *cp_ns = map->cp_ns + delta_us * 1000 + (int64_t)(mac_ps % 1000000) / 1000;
  return true;
}

#ifdef __cplusplus
}
#endif
