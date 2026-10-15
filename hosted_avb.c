/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Coprocessor side of esp_hosted_avb.
 * See README.md for protocol and rationale.
 */

#include "hosted_avb.h"
#include "hosted_avb_peer.h"
#include "hosted_avb_proto.h"

#include <stdatomic.h>
#include <stddef.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#if defined(CONFIG_ESP_HOSTED_CP_FEAT_PEER_DATA)
#include "esp_event.h"
#include "eh_cp_core.h"
#include "eh_cp_feat_wifi_event.h"
#endif

static const char *TAG = "hosted_avb";

/* Replies leave through their own task. In esp-hosted 2.x a custom-RPC
 * callback runs on the pserial task, and a send queues onto that task's
 * own request queue with no timeout: once requests back up, as under a
 * heavy station uplink, a reply sent from the callback blocks the only
 * task that drains the queue, and the coprocessor's RPC and SDIO receive
 * paths stop for good. 3.x bounds that wait but still stalls on it. */
#define REPLY_QUEUE_DEPTH 8
#define REPLY_TASK_STACK 3072
#define REPLY_TASK_PRIORITY 5

/* An ACK carries only the call's esp_err_t; ASSOCIATIONS carries none. */
typedef struct {
  uint32_t msg_id;
  int32_t esp_err;
} hosted_avb_reply_t;

_Static_assert(sizeof(hosted_avb_set_vendor_ie_ack_t) == sizeof(int32_t) &&
                   sizeof(hosted_avb_config_11b_rate_ack_t) == sizeof(int32_t),
               "an ACK is the esp_err_t alone");

static QueueHandle_t s_replies;
static atomic_bool s_associations_queued;
static atomic_uint s_replies_dropped;

static void send_associations(void);
static void reply_task(void *arg);

/* Started on first use, from a task: init runs as a constructor. A reply
 * that races the start is dropped like one that finds the queue full. */
static bool reply_queue_ready(void) {
  static portMUX_TYPE start_lock = portMUX_INITIALIZER_UNLOCKED;
  static bool starting;
  if (s_replies != NULL) {
    return true;
  }
  portENTER_CRITICAL(&start_lock);
  bool claimed = !starting;
  starting = true;
  portEXIT_CRITICAL(&start_lock);
  if (!claimed) {
    return false;
  }
  QueueHandle_t queue =
      xQueueCreate(REPLY_QUEUE_DEPTH, sizeof(hosted_avb_reply_t));
  if (queue == NULL ||
      xTaskCreate(reply_task, "hosted_avb_tx", REPLY_TASK_STACK, queue,
                  REPLY_TASK_PRIORITY, NULL) != pdPASS) {
    ESP_LOGE(TAG, "failed to start the reply task");
    if (queue != NULL) {
      vQueueDelete(queue);
    }
    portENTER_CRITICAL(&start_lock);
    starting = false;
    portEXIT_CRITICAL(&start_lock);
    return false;
  }
  s_replies = queue;
  return true;
}

/* Never waits: a reply that does not fit is dropped. The host logs only
 * a failed ACK and refreshes the associations itself. */
static bool queue_reply(const hosted_avb_reply_t *reply) {
  if (reply_queue_ready() && xQueueSend(s_replies, reply, 0) == pdTRUE) {
    return true;
  }
  unsigned dropped = atomic_fetch_add(&s_replies_dropped, 1) + 1;
  if (dropped == 1 || dropped % 100 == 0) {
    ESP_LOGW(TAG, "reply queue full, %u replies dropped", dropped);
  }
  return false;
}

/* One report pending at a time; the task takes the snapshot when it
 * sends, so a report queued behind a change still carries it. */
static void queue_associations(void) {
  if (atomic_exchange(&s_associations_queued, true)) {
    return;
  }
  hosted_avb_reply_t reply = {.msg_id = HOSTED_AVB_MSG_ASSOCIATIONS};
  if (!queue_reply(&reply)) {
    atomic_store(&s_associations_queued, false);
  }
}

static void reply_task(void *arg) {
  QueueHandle_t queue = arg;
  hosted_avb_reply_t reply;
  for (;;) {
    if (xQueueReceive(queue, &reply, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    if (reply.msg_id == HOSTED_AVB_MSG_ASSOCIATIONS) {
      atomic_store(&s_associations_queued, false);
      send_associations();
      continue;
    }
    esp_err_t sent = hosted_avb_peer_send(
        reply.msg_id, (const uint8_t *)&reply.esp_err, sizeof(reply.esp_err));
    if (sent != ESP_OK) {
      ESP_LOGE(TAG, "ACK 0x%08lx send failed: %s", (unsigned long)reply.msg_id,
               esp_err_to_name(sent));
    }
  }
}

static void set_vendor_ie_callback(uint32_t msg_id, const uint8_t *data,
                                   size_t data_len, void *user) {
  (void)msg_id;
  (void)user;

  hosted_avb_reply_t reply = {.msg_id = HOSTED_AVB_MSG_SET_VENDOR_IE_ACK,
                           .esp_err = ESP_OK};

  if (data == NULL || data_len < sizeof(hosted_avb_set_vendor_ie_t)) {
    ESP_LOGW(TAG, "SET_VENDOR_IE_REQ too short: %zu", data_len);
    reply.esp_err = ESP_ERR_INVALID_SIZE;
    goto send_ack;
  }

  const hosted_avb_set_vendor_ie_t *hdr =
      (const hosted_avb_set_vendor_ie_t *)data;
  uint8_t *vnd_ie = (uint8_t *)(data + sizeof(hosted_avb_set_vendor_ie_t));
  const size_t vnd_ie_len = data_len - sizeof(hosted_avb_set_vendor_ie_t);

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
    reply.esp_err = ESP_ERR_INVALID_SIZE;
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
  reply.esp_err = esp_wifi_set_vendor_ie((bool)hdr->enable,
                                        (wifi_vendor_ie_type_t)hdr->type,
                                        (wifi_vendor_ie_id_t)hdr->idx, vnd_ie);
  if (reply.esp_err != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_set_vendor_ie: %s", esp_err_to_name(reply.esp_err));
  }
  /* Periodic-publish path: no INFO log on success — would spam at the
   * Sync interval (8 Hz for gPTP default). Errors still log. */

send_ack:
  (void)queue_reply(&reply);
}

/* The AP's associations. ftm_radio_association_begin runs before each
 * join, leave or AP stop is forwarded to the host, and
 * ftm_radio_association_complete after, so the list reaches the host
 * behind the event it follows. In esp-hosted 2.x the association
 * lifecycle patch calls them; in 3.x the event handlers below do. */
static portMUX_TYPE s_associations_lock = portMUX_INITIALIZER_UNLOCKED;
static hosted_avb_associations_t s_associations = {
    .version = HOSTED_AVB_ASSOCIATIONS_VERSION, .generation = 1};

uint32_t hosted_avb_radio_boot(void) {
  portENTER_CRITICAL(&s_associations_lock);
  if (s_associations.radio_boot == 0) {
    s_associations.radio_boot = esp_random() | 1U;
  }
  uint32_t radio_boot = s_associations.radio_boot;
  portEXIT_CRITICAL(&s_associations_lock);
  return radio_boot;
}

static void send_associations(void) {
  (void)hosted_avb_radio_boot();
  hosted_avb_associations_t snapshot;
  portENTER_CRITICAL(&s_associations_lock);
  snapshot = s_associations;
  portEXIT_CRITICAL(&s_associations_lock);
  esp_err_t r = hosted_avb_peer_send(HOSTED_AVB_MSG_ASSOCIATIONS,
                                     (const uint8_t *)&snapshot,
                                     sizeof(snapshot));
  if (r != ESP_OK) {
    ESP_LOGW(TAG, "ASSOCIATIONS send failed: %s", esp_err_to_name(r));
  }
}

/* Lifecycle hooks; mac is NULL for an AP stop. Returns the new
 * generation, or 0 for an address no station can have or a full list,
 * either of which leaves the list unchanged. */
uint32_t ftm_radio_association_begin(const uint8_t mac[6], bool joined);
void ftm_radio_association_complete(uint32_t generation, bool queued);

uint32_t ftm_radio_association_begin(const uint8_t mac[6], bool joined) {
  static const uint8_t zero_mac[6] = {0};
  if (mac != NULL && ((mac[0] & 1) || memcmp(mac, zero_mac, 6) == 0)) {
    return 0;
  }
  (void)hosted_avb_radio_boot();
  hosted_avb_associations_t snapshot = {0};
  uint32_t generation = 0;
  portENTER_CRITICAL(&s_associations_lock);
  hosted_avb_associations_t *state = &s_associations;
  int found = -1;
  for (int i = 0; mac != NULL && i < state->count; i++) {
    if (memcmp(state->macs[i], mac, 6) == 0) {
      found = i;
      break;
    }
  }
  bool full = mac != NULL && joined && found < 0 &&
              state->count == HOSTED_AVB_ASSOCIATIONS_MAX;
  if (!full) {
    if (mac == NULL) {
      state->count = 0;
      memset(state->macs, 0, sizeof(state->macs));
    } else if (!joined && found >= 0) {
      state->count--;
      memcpy(state->macs[found], state->macs[state->count], 6);
      memset(state->macs[state->count], 0, 6);
    } else if (joined && found < 0) {
      memcpy(state->macs[state->count++], mac, 6);
    }
    if (++state->generation == 0) {
      ++state->generation;
    }
    state->flags |= HOSTED_AVB_ASSOCIATIONS_FLAG_LIFECYCLE;
    generation = state->generation;
    snapshot = *state;
  }
  portEXIT_CRITICAL(&s_associations_lock);
  if (generation != 0 && hosted_avb_associations_changed) {
    hosted_avb_associations_changed(&snapshot);
  }
  return generation;
}

void ftm_radio_association_complete(uint32_t generation, bool queued) {
  if (hosted_avb_associations_forwarded) {
    hosted_avb_associations_forwarded(generation, queued);
  }
  /* Sent even when the host's event was not queued: the list brings the
   * host back to the radio's associations either way. The event is
   * already queued to the host, so the report still lands behind it. */
  queue_associations();
}

static void associations_request_callback(uint32_t msg_id,
                                          const uint8_t *data,
                                          size_t data_len, void *user) {
  (void)msg_id;
  (void)data;
  (void)data_len;
  (void)user;
  queue_associations();
}

static void config_11b_rate_callback(uint32_t msg_id, const uint8_t *data,
                                     size_t data_len, void *user) {
  (void)msg_id;
  (void)user;
  hosted_avb_reply_t reply = {.msg_id = HOSTED_AVB_MSG_CONFIG_11B_RATE_ACK};
  if (data == NULL || data_len < sizeof(hosted_avb_config_11b_rate_t)) {
    ESP_LOGW(TAG, "CONFIG_11B_RATE_REQ too short: %zu", data_len);
    reply.esp_err = ESP_ERR_INVALID_SIZE;
  } else {
    const hosted_avb_config_11b_rate_t *request =
        (const hosted_avb_config_11b_rate_t *)data;
    reply.esp_err = esp_wifi_config_11b_rate((wifi_interface_t)request->ifx,
                                             request->disable != 0);
    if (reply.esp_err == ESP_OK) {
      ESP_LOGI(TAG, "802.11b rates %s on interface %u",
               request->disable ? "off" : "on", request->ifx);
    } else {
      ESP_LOGE(TAG, "esp_wifi_config_11b_rate: %s",
               esp_err_to_name(reply.esp_err));
    }
  }
  (void)queue_reply(&reply);
}

esp_err_t hosted_avb_init(void) {
  esp_err_t r = hosted_avb_peer_register(HOSTED_AVB_MSG_SET_VENDOR_IE_REQ,
                                         set_vendor_ie_callback, NULL);
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "register SET_VENDOR_IE_REQ failed: %s", esp_err_to_name(r));
    return r;
  }
  r = hosted_avb_peer_register(HOSTED_AVB_MSG_ASSOCIATIONS_REQ,
                               associations_request_callback, NULL);
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "register ASSOCIATIONS_REQ failed: %s", esp_err_to_name(r));
    return r;
  }
  r = hosted_avb_peer_register(HOSTED_AVB_MSG_CONFIG_11B_RATE_REQ,
                               config_11b_rate_callback, NULL);
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "register CONFIG_11B_RATE_REQ failed: %s",
             esp_err_to_name(r));
    return r;
  }
  ESP_LOGI(TAG, "handlers registered (msg_id base 0x%08x)",
           (unsigned)HOSTED_AVB_MSG_BASE);
  return ESP_OK;
}

#if defined(CONFIG_ESP_HOSTED_CP_FEAT_PEER_DATA)
/* esp-hosted 3.x has no association hooks. Its Wi-Fi feature reposts each
 * AP join, leave and stop as an EH_CP_FEAT_WIFI_EVENT, which its RPC
 * feature forwards to the host, all on the default event loop. The raw
 * Wi-Fi event updates the list before the repost is forwarded, and the
 * repost, handled after the forwarder, queues the report behind the
 * host's event. 3.x does not say whether that event was queued, so a
 * repost counts as queued and a change whose repost never came does not.
 * Both handlers run on the event loop task, which owns the pending list. */
#define PENDING_CHANGES_MAX 8

typedef struct {
  uint8_t mac[6];
  bool joined;
  bool ap_stop;
  uint32_t generation;
} pending_change_t;

static pending_change_t s_pending_changes[PENDING_CHANGES_MAX];
static unsigned s_pending_head;
static unsigned s_pending_count;

static void retire_oldest_change(bool queued) {
  pending_change_t *change = &s_pending_changes[s_pending_head];
  s_pending_head = (s_pending_head + 1) % PENDING_CHANGES_MAX;
  s_pending_count--;
  ftm_radio_association_complete(change->generation, queued);
}

static void ap_wifi_event(void *arg, esp_event_base_t base, int32_t event_id,
                          void *event_data) {
  (void)arg;
  (void)base;
  pending_change_t change = {0};
  if (event_id == WIFI_EVENT_AP_STACONNECTED) {
    const wifi_event_ap_staconnected_t *event = event_data;
    memcpy(change.mac, event->mac, 6);
    change.joined = true;
  } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
    const wifi_event_ap_stadisconnected_t *event = event_data;
    memcpy(change.mac, event->mac, 6);
  } else if (event_id == WIFI_EVENT_AP_STOP) {
    change.ap_stop = true;
  } else {
    return;
  }
  change.generation = ftm_radio_association_begin(
      change.ap_stop ? NULL : change.mac, change.joined);
  if (s_pending_count == PENDING_CHANGES_MAX) {
    retire_oldest_change(false);
  }
  s_pending_changes[(s_pending_head + s_pending_count) % PENDING_CHANGES_MAX] =
      change;
  s_pending_count++;
}

static void ap_event_forwarded(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  (void)arg;
  (void)base;
  pending_change_t forwarded = {0};
  if (event_id == EH_CP_FEAT_WIFI_EVT_STA_CONNECTED_TO_ESP_SOFTAP) {
    const wifi_event_ap_staconnected_t *event = event_data;
    memcpy(forwarded.mac, event->mac, 6);
    forwarded.joined = true;
  } else if (event_id == EH_CP_FEAT_WIFI_EVT_STA_DISCONNECTED_FROM_ESP_SOFTAP) {
    const wifi_event_ap_stadisconnected_t *event = event_data;
    memcpy(forwarded.mac, event->mac, 6);
  } else if (event_id == EH_CP_FEAT_WIFI_EVT_SOFTAP_STOPPED) {
    forwarded.ap_stop = true;
  } else {
    return;
  }
  /* Reposts come in event order, so changes ahead of this one lost theirs. */
  while (s_pending_count > 0) {
    const pending_change_t *oldest = &s_pending_changes[s_pending_head];
    bool matches_forwarded =
        oldest->ap_stop == forwarded.ap_stop &&
        oldest->joined == forwarded.joined &&
        (forwarded.ap_stop || memcmp(oldest->mac, forwarded.mac, 6) == 0);
    retire_oldest_change(matches_forwarded);
    if (matches_forwarded) {
      break;
    }
  }
}

/* esp-hosted 3.x starts its features in priority order before the host
 * can connect: peer data at 90, Wi-Fi at 200. This one follows both, so
 * its repost handler runs after the forwarder's. */
static esp_err_t hosted_avb_start(void) {
  esp_err_t r = hosted_avb_init();
  if (r != ESP_OK) {
    return r;
  }
  static const int32_t wifi_events[] = {WIFI_EVENT_AP_STACONNECTED,
                                        WIFI_EVENT_AP_STADISCONNECTED,
                                        WIFI_EVENT_AP_STOP};
  static const int32_t forwarded_events[] = {
      EH_CP_FEAT_WIFI_EVT_STA_CONNECTED_TO_ESP_SOFTAP,
      EH_CP_FEAT_WIFI_EVT_STA_DISCONNECTED_FROM_ESP_SOFTAP,
      EH_CP_FEAT_WIFI_EVT_SOFTAP_STOPPED};
  for (size_t i = 0; i < 3 && r == ESP_OK; i++) {
    r = esp_event_handler_register(WIFI_EVENT, wifi_events[i], ap_wifi_event,
                                   NULL);
    if (r == ESP_OK) {
      r = esp_event_handler_register(EH_CP_FEAT_WIFI_EVENT, forwarded_events[i],
                                     ap_event_forwarded, NULL);
    }
  }
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "AP event handlers: %s", esp_err_to_name(r));
  }
  return r;
}

EH_CP_FEAT_REGISTER(hosted_avb_start, NULL, "hosted_avb", tskNO_AFFINITY, 250);
#else
/* Auto-register so the coprocessor app doesn't need to call us from
 * app_main. Safe at constructor time: esp-hosted 2.x's register API lazy-
 * initializes its mutex and the dispatch table is BSS-zero. Re-calling
 * hosted_avb_init() from user code is harmless. */
__attribute__((constructor)) static void hosted_avb_autoreg(void) {
  (void)hosted_avb_init();
}
#endif
