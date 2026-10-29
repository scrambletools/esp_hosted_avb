/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * esp_hosted_avb wire format. See README.md for rationale and the
 * end-to-end host ↔ coprocessor flow.
 */

#pragma once

#include <stdint.h>

/* Msg-ID base "PTP\0" (0x50545000); low byte is the opcode. The base
 * dates from the component's earlier name, esp_ptp_rpc, and stays so
 * hosts and coprocessors built under either name understand each other.
 * 0xFFFFFFFF is reserved by peer_data_transfer as "invalid". */
#define HOSTED_AVB_MSG_BASE                0x50545000U
#define HOSTED_AVB_MSG_SET_VENDOR_IE_REQ   (HOSTED_AVB_MSG_BASE | 0x01U)
#define HOSTED_AVB_MSG_SET_VENDOR_IE_ACK   (HOSTED_AVB_MSG_BASE | 0x02U)
#define HOSTED_AVB_MSG_ASSOCIATIONS_REQ    (HOSTED_AVB_MSG_BASE | 0x03U)
#define HOSTED_AVB_MSG_ASSOCIATIONS        (HOSTED_AVB_MSG_BASE | 0x04U)
#define HOSTED_AVB_MSG_CONFIG_11B_RATE_REQ (HOSTED_AVB_MSG_BASE | 0x05U)
#define HOSTED_AVB_MSG_CONFIG_11B_RATE_ACK (HOSTED_AVB_MSG_BASE | 0x06U)

/* Vendor IE constants shared between host marshaller, coprocessor
 * handler, and STA parser.
 *
 * Beacon time element (AVB Wireless §2.4.1): OUI 8C:1F:64, type 0x36,
 * sub-ID octets C0 06 (the MA-S 8C-1F-64-36-C plus sub-ID 0x006), then
 * the 76-byte §12.7 Follow_Up, the AP TSF (uint64_t LE µs) and an
 * optional capture bracket (uint16_t LE µs). Its earlier form, type
 * 0x00 under the bare 8C:1F:64 with no sub-ID, is retired: an MA-S
 * holder may not truncate its block to the 24-bit prefix. */
#define PTP_VND_IE_OUI0          0x8C
#define PTP_VND_IE_OUI1          0x1F
#define PTP_VND_IE_OUI2          0x64
#define PTP_VND_IE_OUI_TYPE_AVB_WIRELESS 0x36
#define PTP_VND_IE_SUBID0                0xC0
#define PTP_VND_IE_SUBID1_BEACON_TIME    0x06
#define PTP_VND_IE_SUBID_LEN             2
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
} hosted_avb_set_vendor_ie_t;

/* SET_VENDOR_IE_ACK wire layout. peer_data_transfer confirms only
 * transport delivery, so the call-level esp_err_t comes back here. */
typedef struct __attribute__((packed)) {
  int32_t esp_err;
} hosted_avb_set_vendor_ie_ack_t;

/* CONFIG_11B_RATE_REQ wire layout: esp_wifi_config_11b_rate() on the
 * coprocessor, which esp-hosted does not forward. With 802.11b off the
 * AP sends beacons, management and group addressed frames at 6 Mbps
 * OFDM instead of 1 Mbps DSSS. The radio accepts it only between
 * esp_wifi_init() and esp_wifi_start(), so the host waits for the ACK
 * before it starts Wi-Fi. */
typedef struct __attribute__((packed)) {
  uint8_t ifx;      /* wifi_interface_t */
  uint8_t disable;  /* 1 = 802.11b rates off */
  uint8_t reserved[2];
} hosted_avb_config_11b_rate_t;

/* CONFIG_11B_RATE_ACK wire layout: the call's esp_err_t. */
typedef struct __attribute__((packed)) {
  int32_t esp_err;
} hosted_avb_config_11b_rate_ack_t;

/* ASSOCIATIONS wire layout: the stations associated with the
 * coprocessor's access point, sent after each join, leave and AP stop
 * has been forwarded to the host, and in answer to ASSOCIATIONS_REQ
 * (which carries only the version byte). The host reconciles its gPTP
 * peers to it, so it serves only the stations the radio has. radio_boot
 * is random per coprocessor boot and generation advances with every
 * change; neither is ever 0. LIFECYCLE is set once the coprocessor's
 * esp-hosted reports association changes (the association lifecycle
 * patch); until then the list cannot be trusted and the host goes by
 * its own events. */
#define HOSTED_AVB_ASSOCIATIONS_VERSION        1
#define HOSTED_AVB_ASSOCIATIONS_MAX            16
#define HOSTED_AVB_ASSOCIATIONS_FLAG_LIFECYCLE 0x01

typedef struct __attribute__((packed)) {
  uint8_t version;
  uint8_t flags;
  uint8_t count;
  uint8_t reserved;
  uint32_t radio_boot;
  uint32_t generation;
  uint8_t macs[HOSTED_AVB_ASSOCIATIONS_MAX][6];
} hosted_avb_associations_t;
