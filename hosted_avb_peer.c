/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Peer data calls for the esp-hosted host and coprocessor.
 */

#include "hosted_avb_peer.h"

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if defined(CONFIG_ESP_HOSTED_CP_FEAT_PEER_DATA)
#include "eh_cp_feat_peer_data.h"
#define PEER_SEND eh_cp_feat_peer_data_send
#define PEER_REGISTER eh_cp_feat_peer_data_register_callback
#else
#include "eh_host_feat_peer_data.h"
#define PEER_SEND eh_host_peer_data_send
#define PEER_REGISTER eh_host_peer_data_register
#endif

esp_err_t hosted_avb_peer_send(uint32_t msg_id, const uint8_t *data,
                               size_t data_len) {
  return PEER_SEND(msg_id, data, data_len);
}

esp_err_t hosted_avb_peer_register(uint32_t msg_id, hosted_avb_peer_cb_t cb,
                                   void *user) {
  esp_err_t result = PEER_REGISTER(msg_id, cb, user);
#if defined(CONFIG_ESP_HOSTED_CP_FEAT_PEER_DATA)
  const TickType_t step = pdMS_TO_TICKS(10);
  int waited_ms = 0;
  while (result == ESP_ERR_INVALID_STATE &&
         waited_ms < HOSTED_AVB_PEER_START_WAIT_MS &&
         xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    vTaskDelay(step);
    waited_ms += 10;
    result = PEER_REGISTER(msg_id, cb, user);
  }
#endif
  return result;
}
