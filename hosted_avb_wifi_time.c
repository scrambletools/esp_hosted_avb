/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Wi-Fi time on the host (see hosted_avb_wifi_time.h).
 */
#include "hosted_avb_wifi_time.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hosted_avb_clock_link.h"
#include "hosted_avb_peer.h"
#include "hosted_avb_proto.h"

#define REPORT_PERIOD_US (10 * 1000 * 1000LL)
#define GROUP_SIZE 8 /* about a second of measurements */

static const char *TAG = "wifi_time";

static QueueHandle_t s_messages;
static TaskHandle_t s_task;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static hosted_avb_wifi_time_sample_t s_latest;
static bool s_have_latest;
static hosted_avb_wifi_time_callback_t s_callback;
static void *s_callback_context;

/* One reporting window. */
typedef struct {
  uint32_t results[HOSTED_AVB_WIFI_TIME_RESULTS];
  uint32_t unconverted;
  uint32_t errors;
  double error_sum, error_squares, error_low, error_high;
  double group_sum, group_low, group_high;
  uint32_t group_count, groups;
  double rtt_sum, rate_sum;
  uint32_t map_uncertainty_max;
  int32_t map_shift_max;
  /* Sessions by measurements and Follow_Ups received: 2/2, 2/1, 2/0,
   * 1/1, 1/0, anything else. */
  uint32_t shapes[6];
  uint32_t refused, no_valid, other_failed;
  /* Measurements from single-measurement sessions, on their own. */
  uint32_t singles, singles_from_sessions, singles_held;
  double single_sum, single_squares;
} window_t;

static void count_session(window_t *window,
                          const hosted_avb_wifi_time_t *message) {
  window->refused += message->refused;
  if (message->result == HOSTED_AVB_WIFI_TIME_NO_REPORT ||
      message->result == HOSTED_AVB_WIFI_TIME_NOT_STARTED ||
      (message->result == HOSTED_AVB_WIFI_TIME_NO_MAP && !message->entries))
    return;
  if (message->ftm_status != 0) {
    if (message->ftm_status == 5) /* FTM_STATUS_NO_VALID_MSMT */
      window->no_valid++;
    else
      window->other_failed++;
    return;
  }
  int shape = 5;
  if (message->entries == 2 && message->follow_ups <= 2)
    shape = 2 - message->follow_ups;
  else if (message->entries == 1 && message->follow_ups <= 1)
    shape = 4 - message->follow_ups;
  window->shapes[shape]++;
}

static void on_message(uint32_t msg_id, const uint8_t *data, size_t length,
                       void *user) {
  if (msg_id == HOSTED_AVB_MSG_WIFI_TIME &&
      length == sizeof(hosted_avb_wifi_time_t))
    xQueueSend(s_messages, data, 0);
}

static void add_error(window_t *window, double error_ns,
                      const hosted_avb_wifi_time_t *message) {
  if (!window->errors || error_ns < window->error_low)
    window->error_low = error_ns;
  if (!window->errors || error_ns > window->error_high)
    window->error_high = error_ns;
  window->errors++;
  window->error_sum += error_ns;
  window->error_squares += error_ns * error_ns;
  window->rtt_sum += message->rtt_ps / 1000.0;
  window->rate_sum += message->rate_ppb / 1000.0;
  if (message->map_uncertainty_ns > window->map_uncertainty_max)
    window->map_uncertainty_max = message->map_uncertainty_ns;
  int32_t shift = message->map_shift_ns < 0 ? -message->map_shift_ns
                                            : message->map_shift_ns;
  if (shift > window->map_shift_max)
    window->map_shift_max = shift;
  /* Means of about a second, as a servo would follow them. */
  window->group_sum += error_ns;
  if (++window->group_count == GROUP_SIZE) {
    double mean = window->group_sum / GROUP_SIZE;
    if (!window->groups || mean < window->group_low)
      window->group_low = mean;
    if (!window->groups || mean > window->group_high)
      window->group_high = mean;
    window->groups++;
    window->group_sum = 0;
    window->group_count = 0;
  }
}

static void report_sessions(const window_t *window) {
  const uint32_t *k = window->shapes;
  double mean = window->singles ? window->single_sum / window->singles : 0;
  double variance =
      window->singles ? window->single_squares / window->singles - mean * mean
                      : 0;
  ESP_LOGI(TAG,
           "WIFISESS,measurements/Follow_Ups 2/2 %" PRIu32 " 2/1 %" PRIu32
           " 2/0 %" PRIu32 " 1/1 %" PRIu32 " 1/0 %" PRIu32 " other %" PRIu32
           ",refused %" PRIu32 ",no valid %" PRIu32 ",other failures %" PRIu32
           ",single used %" PRIu32 " (rate from sessions %" PRIu32
           " held %" PRIu32 ") error mean %.0f sd %.0f",
           k[0], k[1], k[2], k[3], k[4], k[5], window->refused,
           window->no_valid, window->other_failed, window->singles,
           window->singles_from_sessions, window->singles_held, mean,
           sqrt(variance > 0 ? variance : 0));
}

static void report(const window_t *window) {
  const uint32_t *r = window->results;
  uint32_t sessions = 0;
  for (int i = 0; i < HOSTED_AVB_WIFI_TIME_RESULTS; i++)
    sessions += r[i];
  if (!sessions) {
    ESP_LOGW(TAG, "WIFITIME,no sessions from the coprocessor");
    return;
  }
  if (!window->errors) {
    ESP_LOGW(TAG,
             "WIFITIME,%" PRIu32 " of %" PRIu32 " sessions measured,no report "
             "%" PRIu32 ",failed %" PRIu32 ",no Follow_Up %" PRIu32
             ",rejected %" PRIu32 ",no map %" PRIu32 ",not started %" PRIu32
             ",not converted %" PRIu32,
             r[HOSTED_AVB_WIFI_TIME_OBSERVED], sessions,
             r[HOSTED_AVB_WIFI_TIME_NO_REPORT], r[HOSTED_AVB_WIFI_TIME_FAILED],
             r[HOSTED_AVB_WIFI_TIME_NO_FOLLOW_UP],
             r[HOSTED_AVB_WIFI_TIME_REJECTED], r[HOSTED_AVB_WIFI_TIME_NO_MAP],
             r[HOSTED_AVB_WIFI_TIME_NOT_STARTED], window->unconverted);
    return;
  }
  double count = window->errors;
  double mean = window->error_sum / count;
  double variance = window->error_squares / count - mean * mean;
  ESP_LOGI(TAG,
           "WIFITIME,%" PRIu32 " of %" PRIu32 " sessions,error mean %.0f ns "
           "sd %.0f p-p %.0f,1 s means p-p %.0f,rtt %.1f ns,rate %.3f ppm,"
           "map +-%" PRIu32 " ns shift %" PRId32 ",no report %" PRIu32
           ",failed %" PRIu32 ",no Follow_Up %" PRIu32 ",rejected %" PRIu32
           ",no map %" PRIu32 ",not converted %" PRIu32,
           window->errors, sessions, mean, sqrt(variance > 0 ? variance : 0),
           window->error_high - window->error_low,
           window->groups >= 2 ? window->group_high - window->group_low : 0.0,
           window->rtt_sum / count, window->rate_sum / count,
           window->map_uncertainty_max, window->map_shift_max,
           r[HOSTED_AVB_WIFI_TIME_NO_REPORT], r[HOSTED_AVB_WIFI_TIME_FAILED],
           r[HOSTED_AVB_WIFI_TIME_NO_FOLLOW_UP],
           r[HOSTED_AVB_WIFI_TIME_REJECTED], r[HOSTED_AVB_WIFI_TIME_NO_MAP],
           window->unconverted);
}

static void wifi_time_task(void *arg) {
  window_t window = {0};
  uint8_t source_port[10] = {0};
  uint16_t time_base = 0;
  int64_t next_report = esp_timer_get_time() + REPORT_PERIOD_US;
  for (;;) {
    hosted_avb_wifi_time_t message;
    int64_t now = esp_timer_get_time();
    TickType_t wait =
        now >= next_report ? 0 : pdMS_TO_TICKS((next_report - now) / 1000) + 1;
    if (xQueueReceive(s_messages, &message, wait) == pdTRUE &&
        message.version == HOSTED_AVB_WIFI_TIME_VERSION &&
        message.result < HOSTED_AVB_WIFI_TIME_RESULTS) {
      window.results[message.result]++;
      count_session(&window, &message);
      int64_t host_ns;
      if (message.result != HOSTED_AVB_WIFI_TIME_OBSERVED) {
        /* counted */
      } else if (!hosted_avb_clock_link_to_host(message.cp_ns, &host_ns)) {
        window.unconverted++;
      } else {
        if (memcmp(source_port, message.source_port, 10) ||
            time_base != message.time_base) {
          memcpy(source_port, message.source_port, 10);
          time_base = message.time_base;
          const uint8_t *id = message.source_port;
          ESP_LOGI(TAG,
                   "Wi-Fi time from %02x%02x%02x.%02x%02x.%02x%02x%02x port %u, "
                   "domain %u, time base %u, %u frames granted",
                   id[0], id[1], id[2], id[3], id[4], id[5], id[6], id[7],
                   (unsigned)(id[8] << 8 | id[9]), message.domain,
                   message.time_base, message.granted_frames);
        }
        double error_ns = (double)(message.reference_ns - host_ns);
        add_error(&window, error_ns, &message);
        if (message.follow_ups == 1) {
          window.singles++;
          window.single_sum += error_ns;
          window.single_squares += error_ns * error_ns;
          if (message.rate_source == HOSTED_AVB_WIFI_TIME_RATE_SESSIONS)
            window.singles_from_sessions++;
          else if (message.rate_source == HOSTED_AVB_WIFI_TIME_RATE_HELD)
            window.singles_held++;
        }
        hosted_avb_wifi_time_sample_t sample = {
            .host_ns = host_ns,
            .reference_ns = message.reference_ns,
            .received_us = esp_timer_get_time(),
            .rtt_ns = message.rtt_ps / 1000,
            .time_base = message.time_base,
            .domain = message.domain,
        };
        memcpy(sample.source_port, message.source_port, 10);
        portENTER_CRITICAL(&s_lock);
        s_latest = sample;
        s_have_latest = true;
        hosted_avb_wifi_time_callback_t callback = s_callback;
        void *context = s_callback_context;
        portEXIT_CRITICAL(&s_lock);
        if (callback)
          callback(&sample, context);
      }
    }
    if (esp_timer_get_time() >= next_report) {
      report(&window);
      report_sessions(&window);
      window = (window_t){0};
      next_report += REPORT_PERIOD_US;
    }
  }
}

esp_err_t hosted_avb_wifi_time_start(void) {
  if (s_task)
    return ESP_OK;
  s_messages = xQueueCreate(16, sizeof(hosted_avb_wifi_time_t));
  if (!s_messages)
    return ESP_ERR_NO_MEM;
  esp_err_t err =
      hosted_avb_peer_register(HOSTED_AVB_MSG_WIFI_TIME, on_message, NULL);
  if (err != ESP_OK)
    return err;
  /* Core 0, as the clock link: unpinned it starved behind an endpoint
   * talker on core 1 and the coprocessor's sessions were dropped. */
  if (xTaskCreatePinnedToCore(wifi_time_task, "wifi_time", 4096, NULL, 5,
                              &s_task, 0) != pdPASS)
    return ESP_ERR_NO_MEM;
  ESP_LOGI(TAG, "Receiving the coprocessor's Wi-Fi time");
  return ESP_OK;
}

bool hosted_avb_wifi_time_latest(hosted_avb_wifi_time_sample_t *sample) {
  portENTER_CRITICAL(&s_lock);
  bool have = s_have_latest;
  if (have)
    *sample = s_latest;
  portEXIT_CRITICAL(&s_lock);
  return have;
}

void hosted_avb_wifi_time_set_callback(hosted_avb_wifi_time_callback_t callback,
                                       void *context) {
  portENTER_CRITICAL(&s_lock);
  s_callback = callback;
  s_callback_context = context;
  portEXIT_CRITICAL(&s_lock);
}
