/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Host side of esp_hosted_avb: the coprocessor's firmware update, written
 * over esp-hosted's OTA calls. The two functions have the signatures of
 * esp_avb_coprocessor_fw_t's store and activate, so a host offers the
 * coprocessor's firmware to ATDECC controllers with:
 *
 *   static const esp_avb_coprocessor_fw_t firmware = {
 *       .name = "Coprocessor firmware",
 *       .max_length = HOSTED_AVB_COPROCESSOR_SLOT_SIZE,
 *       .store = hosted_avb_coprocessor_fw_store,
 *       .activate = hosted_avb_coprocessor_fw_activate,
 *   };
 *   esp_avb_register_coprocessor_fw(&firmware);
 */

#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The coprocessor's OTA slot: two 1792 KB slots in its partition table. */
#define HOSTED_AVB_COPROCESSOR_SLOT_SIZE (1792 * 1024)

/* Checks image is an app for the coprocessor's chip from the project
 * CONFIG_HOSTED_AVB_COPROCESSOR_PROJECT, as long as its header says,
 * then writes it to the coprocessor's spare OTA slot; the coprocessor
 * checks it again as it closes the slot. progress gets permille, from
 * 10 to 990. A failure leaves the coprocessor on its running image. */
esp_err_t hosted_avb_coprocessor_fw_store(const uint8_t *image,
                                          uint32_t length,
                                          void (*progress)(uint16_t permille),
                                          void *context);

/* Switches the coprocessor to the stored image; it restarts 2 s later. */
esp_err_t hosted_avb_coprocessor_fw_activate(void *context);

#ifdef __cplusplus
}
#endif
