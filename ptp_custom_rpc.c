/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Coprocessor-side handler for the PTP custom-RPC channel.
 * See README.md for protocol and rationale.
 */

#include "ptp_custom_rpc.h"
#include "ptp_rpc_proto.h"

#include <stddef.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"

/* esp_hosted_peer_data.h lives inside upstream
 * esp-hosted-mcu/slave/main and is not exported as a component
 * header — forward-declare locally to stay decoupled. */
extern esp_err_t esp_hosted_send_custom_data(uint32_t msg_id,
                                             const uint8_t *data,
                                             size_t data_len);
extern esp_err_t esp_hosted_register_custom_callback(
    uint32_t msg_id,
    void (*cb)(uint32_t, const uint8_t *, size_t, void *),
    void *user);

static const char *TAG = "ptp";

static void set_vendor_ie_callback(uint32_t msg_id, const uint8_t *data,
                                   size_t data_len, void *user) {
  (void)msg_id;
  (void)user;

  ptp_rpc_set_vendor_ie_ack_t ack = {.esp_err = ESP_OK};

  if (data == NULL || data_len < sizeof(ptp_rpc_set_vendor_ie_t)) {
    ESP_LOGW(TAG, "SET_VENDOR_IE_REQ too short: %zu", data_len);
    ack.esp_err = ESP_ERR_INVALID_SIZE;
    goto send_ack;
  }

  const ptp_rpc_set_vendor_ie_t *hdr = (const ptp_rpc_set_vendor_ie_t *)data;
  uint8_t *vnd_ie = (uint8_t *)(data + sizeof(ptp_rpc_set_vendor_ie_t));
  const size_t vnd_ie_len = data_len - sizeof(ptp_rpc_set_vendor_ie_t);

  static uint32_t s_calls = 0;
  ++s_calls;
  if ((s_calls % 25) == 1) {
    ESP_LOGI(TAG,
             "set_vendor_ie cb #%u: en=%u type=%u idx=%u vnd_ie_len=%u "
             "oui_type=0x%02x",
             (unsigned)s_calls, hdr->enable, hdr->type, hdr->idx,
             (unsigned)vnd_ie_len,
             (vnd_ie_len >= 6) ? vnd_ie[5] : 0xff);
  }

  /* vendor_ie_data_t needs at least its 6-byte fixed header. */
  if (hdr->enable && vnd_ie_len < 6) {
    ESP_LOGW(TAG, "SET_VENDOR_IE_REQ vnd_ie too short: %zu", vnd_ie_len);
    ack.esp_err = ESP_ERR_INVALID_SIZE;
    goto send_ack;
  }

  /* Beacon time element patch: the host leaves the AP TSF zero after the
   * §12.7 Follow_Up. Fill it with esp_wifi_get_tsf_time(WIFI_IF_AP) (LE
   * µs) here, right before the element reaches the wifi blob, so the
   * AP-TSF marker is captured atomically with the beacon carrying this
   * BTC (preciseOriginTimestamp). The host cannot read the AP TSF itself
   * (its remote esp_wifi_get_tsf_time returns -1), so only the
   * coprocessor can supply it. The earlier FOLLOWUP form carries the TSF
   * in its last PTP_VND_IE_TSF_MAPPING_PAYLOAD_LEN bytes. */
  size_t tsf_offset = 0;
  if (hdr->enable && vnd_ie[2] == PTP_VND_IE_OUI0 &&
      vnd_ie[3] == PTP_VND_IE_OUI1 && vnd_ie[4] == PTP_VND_IE_OUI2) {
    if (vnd_ie[5] == PTP_VND_IE_OUI_TYPE_AVB_WIRELESS &&
        vnd_ie_len >= 6 + PTP_VND_IE_SUBID_LEN &&
        vnd_ie[6] == PTP_VND_IE_SUBID0 &&
        vnd_ie[7] == PTP_VND_IE_SUBID1_BEACON_TIME) {
      tsf_offset = 6 + PTP_VND_IE_SUBID_LEN + PTP_VND_IE_FOLLOW_UP_LEN;
    } else if (vnd_ie[5] == PTP_VND_IE_OUI_TYPE_FOLLOWUP &&
               vnd_ie_len >= 6 + PTP_VND_IE_TSF_MAPPING_PAYLOAD_LEN) {
      tsf_offset = vnd_ie_len - PTP_VND_IE_TSF_MAPPING_PAYLOAD_LEN;
    }
  }
  if (tsf_offset != 0 &&
      tsf_offset + PTP_VND_IE_TSF_MAPPING_PAYLOAD_LEN <= vnd_ie_len) {
    int64_t tsf_us = esp_wifi_get_tsf_time(WIFI_IF_AP);
    uint64_t v = (uint64_t)tsf_us;
    uint8_t *tsf = vnd_ie + tsf_offset;
    for (int i = 0; i < PTP_VND_IE_TSF_MAPPING_PAYLOAD_LEN; ++i) {
      tsf[i] = (uint8_t)((v >> (8 * i)) & 0xff);
    }
    static uint32_t s_tsf_fills = 0;
    if ((++s_tsf_fills % 25) == 1) {
      ESP_LOGI(TAG,
               "Beacon time element AP-TSF patched #%u: tsf=%lld us idx=%u",
               (unsigned)s_tsf_fills, (long long)tsf_us, hdr->idx);
    }
  }

  /* esp_wifi_set_vendor_ie returns ESP_ERR_INVALID_ARG (wifi:"the
   * vendor ie has been setted, clear it before setting again") when a
   * Vendor IE is already installed at the same (type, idx) slot. For
   * the gPTP FollowUpInformation publish use case the IE payload
   * changes every Sync interval, so we clear-then-set on every
   * install. The clear is a no-op the first time and cheap thereafter. */
  if (hdr->enable) {
    (void)esp_wifi_set_vendor_ie(false, (wifi_vendor_ie_type_t)hdr->type,
                                 (wifi_vendor_ie_id_t)hdr->idx, NULL);
  }
  ack.esp_err = esp_wifi_set_vendor_ie((bool)hdr->enable,
                                       (wifi_vendor_ie_type_t)hdr->type,
                                       (wifi_vendor_ie_id_t)hdr->idx, vnd_ie);
  if (ack.esp_err != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_set_vendor_ie: %s", esp_err_to_name(ack.esp_err));
  }
  /* Periodic-publish path: no INFO log on success — would spam at the
   * Sync interval (8 Hz for gPTP default). Errors still log. */

send_ack: {
  esp_err_t s = esp_hosted_send_custom_data(PTP_RPC_MSG_SET_VENDOR_IE_ACK,
                                            (const uint8_t *)&ack, sizeof(ack));
  if (s != ESP_OK) {
    ESP_LOGE(TAG, "SET_VENDOR_IE_ACK send failed: %s", esp_err_to_name(s));
  }
}
}

esp_err_t ptp_custom_rpc_init(void) {
  esp_err_t r = esp_hosted_register_custom_callback(
      PTP_RPC_MSG_SET_VENDOR_IE_REQ, set_vendor_ie_callback, NULL);
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "register SET_VENDOR_IE_REQ failed: %s", esp_err_to_name(r));
    return r;
  }
  ESP_LOGI(TAG, "PTP custom RPC handler registered (msg_id=0x%08x)",
           (unsigned)PTP_RPC_MSG_SET_VENDOR_IE_REQ);
  return ESP_OK;
}

/* Auto-register so the coprocessor app doesn't need to call us from
 * app_main. Safe at constructor time: upstream's register API lazy-
 * initializes its mutex and the dispatch table is BSS-zero. Re-calling
 * ptp_custom_rpc_init() from user code is harmless. */
__attribute__((constructor)) static void ptp_custom_rpc_autoreg(void) {
  (void)ptp_custom_rpc_init();
}
