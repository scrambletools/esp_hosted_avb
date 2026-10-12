/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Coprocessor-side handler init for the PTP custom-RPC channel.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "ptp_rpc_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ptp_custom_rpc_init(void);

/* This coprocessor boot's identity in the ASSOCIATIONS message, random
 * and never 0. */
uint32_t ptp_rpc_radio_boot(void);

/* Optional hooks for other coprocessor components that follow the AP's
 * associations. changed runs after each join, leave or AP stop with the
 * new state, before the host is told; forwarded runs once the host's
 * notification is queued, or failed to be. Both may run in the Wi-Fi
 * event task. */
void ptp_rpc_associations_changed(const ptp_rpc_associations_t *associations)
    __attribute__((weak));
void ptp_rpc_associations_forwarded(uint32_t generation, bool queued)
    __attribute__((weak));

#ifdef __cplusplus
}
#endif
