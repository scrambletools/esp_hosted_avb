/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Coprocessor side of esp_hosted_avb.
 */

#pragma once

#include <stdbool.h>
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
 * notification is queued, or failed to be. Both may run in the Wi-Fi
 * event task. */
void hosted_avb_associations_changed(
    const hosted_avb_associations_t *associations) __attribute__((weak));
void hosted_avb_associations_forwarded(uint32_t generation, bool queued)
    __attribute__((weak));

#ifdef __cplusplus
}
#endif
