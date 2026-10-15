/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Peer data calls of the esp-hosted this side is built with, so callers
 * build unchanged against esp-hosted 2.x or 3.x, host or coprocessor.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*hosted_avb_peer_cb_t)(uint32_t msg_id, const uint8_t *data,
                                     size_t data_len, void *user);

/* Sends one message to the peer. */
esp_err_t hosted_avb_peer_send(uint32_t msg_id, const uint8_t *data,
                               size_t data_len);

/* Registers the callback for msg_id. A 3.x coprocessor refuses callbacks
 * until its peer data feature has started; called from a task, this waits
 * up to HOSTED_AVB_PEER_START_WAIT_MS for it. */
esp_err_t hosted_avb_peer_register(uint32_t msg_id, hosted_avb_peer_cb_t cb,
                                   void *user);

#define HOSTED_AVB_PEER_START_WAIT_MS 10000

#ifdef __cplusplus
}
#endif
