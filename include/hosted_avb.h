/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Coprocessor side of esp_hosted_avb.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "hosted_avb_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t hosted_avb_init(void);

/* This coprocessor boot's identity in the ASSOCIATIONS message, random
 * and never 0. */
uint32_t hosted_avb_radio_boot(void);

/* Optional hooks for other coprocessor components that follow the AP's
 * associations. changed runs after each join, leave or AP stop with the
 * new state, before the host is told; forwarded runs once the host's
 * notification is queued, or failed to be. Both run on the default
 * event loop task. */
void hosted_avb_associations_changed(
    const hosted_avb_associations_t *associations) __attribute__((weak));
void hosted_avb_associations_forwarded(uint32_t generation, bool queued)
    __attribute__((weak));

/* Optional hook for the beacon time element: the host sends its §12.7
 * Follow_Up with gPTP time from when it built the element, and the
 * coprocessor appends the AP TSF when the element arrives, a host
 * message apart. A component that knows gPTP time on this chip writes
 * both for one instant instead: the Follow_Up (follow_up_len octets,
 * its preciseOriginTimestamp, correctionField and
 * cumulativeScaledRateOffset) and the AP TSF (8 octets, little-endian
 * µs). It runs on each install, before the element reaches the radio;
 * false leaves the host's Follow_Up and only adds the TSF. */
bool hosted_avb_beacon_time_fill(uint8_t *follow_up, size_t follow_up_len,
                                 uint8_t tsf_le[8]) __attribute__((weak));

#ifdef __cplusplus
}
#endif
