/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * PTP custom-RPC wire format. See README.md for rationale and the
 * end-to-end host ↔ coprocessor flow.
 */

#pragma once

#include <stdint.h>

/* Msg-ID base "PTP\0" (0x50545000); low byte is the opcode.
 * 0xFFFFFFFF is reserved by peer_data_transfer as "invalid". */
#define PTP_RPC_MSG_BASE              0x50545000U
#define PTP_RPC_MSG_SET_VENDOR_IE_REQ (PTP_RPC_MSG_BASE | 0x01U)
#define PTP_RPC_MSG_SET_VENDOR_IE_ACK (PTP_RPC_MSG_BASE | 0x02U)

/* Vendor IE constants shared between host marshaller, coprocessor RPC
 * handler, and STA parser.
 *
 * Beacon time element (AVB Wireless §2.4.1): OUI 8C:1F:64, type 0x36,
 * sub-ID octets C0 06 (the MA-S 8C-1F-64-36-C plus sub-ID 0x006), then
 * the 76-byte §12.7 Follow_Up, the AP TSF (uint64_t LE µs) and an
 * optional capture bracket (uint16_t LE µs).
 *
 * FOLLOWUP (0x00) is the earlier form, the Follow_Up and AP TSF with no
 * sub-ID. Stations accept it during migration. */
#define PTP_VND_IE_OUI0          0x8C
#define PTP_VND_IE_OUI1          0x1F
#define PTP_VND_IE_OUI2          0x64
#define PTP_VND_IE_OUI_TYPE_AVB_WIRELESS 0x36
#define PTP_VND_IE_SUBID0                0xC0
#define PTP_VND_IE_SUBID1_BEACON_TIME    0x06
#define PTP_VND_IE_SUBID_LEN             2
#define PTP_VND_IE_OUI_TYPE_FOLLOWUP     0x00
#define PTP_VND_IE_FOLLOW_UP_LEN            76
#define PTP_VND_IE_TSF_MAPPING_PAYLOAD_LEN  8   /* uint64_t LE µs */
#define PTP_VND_IE_CAPTURE_BRACKET_LEN      2   /* uint16_t LE µs */
#define PTP_VND_IE_CAPTURE_BRACKET_UNKNOWN  0xFFFF

/* SET_VENDOR_IE_REQ wire layout: this 4-byte header followed by a
 * full vendor_ie_data_t buffer (element_id + length + OUI + oui_type
 * + payload), byte-identical to esp_wifi_set_vendor_ie()'s arg.
 * Reserved byte keeps the trailing IE buffer 4-byte aligned. */
typedef struct __attribute__((packed)) {
  uint8_t enable;   /* 0=remove, 1=install/update */
  uint8_t type;     /* wifi_vendor_ie_type_t */
  uint8_t idx;      /* wifi_vendor_ie_id_t */
  uint8_t reserved;
} ptp_rpc_set_vendor_ie_t;

/* SET_VENDOR_IE_ACK wire layout. peer_data_transfer confirms only
 * transport delivery, so the call-level esp_err_t comes back here. */
typedef struct __attribute__((packed)) {
  int32_t esp_err;
} ptp_rpc_set_vendor_ie_ack_t;
