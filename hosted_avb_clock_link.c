/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Clock link between the host and the coprocessor (see
 * hosted_avb_clock_link.h). Per edge: the host sends ARM with a sequence,
 * the coprocessor arms its capture and answers ARM_ACK, the host pulses
 * the link and reads its PTP clock against its own latch of the edge, and
 * the coprocessor sends EDGE with its latch and a bracketed read of its
 * Wi-Fi MAC clock. The host keeps the pairs and logs a fit of the
 * coprocessor's timer against its PTP clock.
 */
#include "hosted_avb_clock_link.h"

#include "sdkconfig.h"

#if CONFIG_HOSTED_AVB_CLOCK_LINK

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/gpio_etm.h"
#include "driver/gptimer.h"
#include "driver/gptimer_etm.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_etm.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hosted_avb_peer.h"
#include "hosted_avb_proto.h"

#define TICK_NS 25 /* the 40 MHz capture timers */

static const char *TAG = "clock_link";

/* hosted_avb_timer_bracket.c, built inside the GPTimer driver. */
extern void hosted_avb_timer_bracket(gptimer_handle_t timer,
                                     volatile const uint32_t *reg,
                                     uint64_t *before, uint32_t *value,
                                     uint64_t *after);

/* A free-running 40 MHz timer whose capture register latches each rising
 * edge on gpio, in hardware (GPIO ETM event to timer capture task). */
static esp_err_t new_edge_timer(int gpio, gptimer_handle_t *timer) {
  gptimer_config_t config = {
      .clk_src = GPTIMER_CLK_SRC_PLL_F80M,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000000 / TICK_NS,
  };
  ESP_RETURN_ON_ERROR(gptimer_new_timer(&config, timer), TAG, "timer");
  esp_etm_channel_config_t channel_config = {0};
  esp_etm_channel_handle_t channel;
  ESP_RETURN_ON_ERROR(esp_etm_new_channel(&channel_config, &channel), TAG,
                      "ETM channel");
  gpio_etm_event_config_t edge_config = {.edge = GPIO_ETM_EVENT_EDGE_POS};
  esp_etm_event_handle_t edge;
  ESP_RETURN_ON_ERROR(gpio_new_etm_event(&edge_config, &edge), TAG,
                      "GPIO event");
  ESP_RETURN_ON_ERROR(gpio_etm_event_bind_gpio(edge, gpio), TAG, "bind");
  gptimer_etm_task_config_t capture_config = {
      .task_type = GPTIMER_ETM_TASK_CAPTURE};
  esp_etm_task_handle_t capture;
  ESP_RETURN_ON_ERROR(gptimer_new_etm_task(*timer, &capture_config, &capture),
                      TAG, "capture task");
  ESP_RETURN_ON_ERROR(esp_etm_channel_connect(channel, edge, capture), TAG,
                      "connect");
  ESP_RETURN_ON_ERROR(gptimer_enable(*timer), TAG, "enable");
  ESP_RETURN_ON_ERROR(gptimer_start(*timer), TAG, "start");
  ESP_RETURN_ON_ERROR(esp_etm_channel_enable(channel), TAG, "ETM enable");
  return ESP_OK;
}

/* ---------------------------------------------------------------- host */
#if CONFIG_ESP_HOSTED_HOST && CONFIG_IDF_TARGET_ESP32P4

#include "soc/emac_ptp_struct.h"

#define HOST_GPIO CONFIG_HOSTED_AVB_CLOCK_LINK_HOST_GPIO
#define REPLY_TIMEOUT_MS 100
#define FIT_SIZE 32
#define REPORT_PERIOD_US (10 * 1000 * 1000LL)
#define BRACKET_MAX_TICKS 40 /* 1 us */

static gptimer_handle_t s_host_timer;
static QueueHandle_t s_acks, s_edges;
static uint32_t s_host_boot;
static portMUX_TYPE s_latest_lock = portMUX_INITIALIZER_UNLOCKED;
static hosted_avb_clock_pair_t s_latest;
static bool s_have_latest;
static TaskHandle_t s_link_task;

#define FIT_MIN_PAIRS 8
#define FIT_MAX_AGE_US (2 * 1000 * 1000LL)
#define FIT_MAX_SPAN_NS (20 * 1000 * 1000 * 1000LL)
#define STEP_LIMIT_NS 2000.0

/* The host PTP clock against the coprocessor timer, fitted over the last
 * pairs: host = anchor_host + intercept + slope * (cp - anchor_cp). */
typedef struct {
  bool valid;
  int count;
  uint32_t coprocessor_boot;
  uint64_t anchor_ticks;
  int64_t anchor_host_ns;
  double slope, intercept, rms, peak_to_peak;
  int64_t updated_us;
} clock_fit_t;
static clock_fit_t s_fit;

static void on_reply(uint32_t msg_id, const uint8_t *data, size_t length,
                     void *user) {
  if (msg_id == HOSTED_AVB_MSG_CLOCK_LINK_ARM_ACK &&
      length == sizeof(hosted_avb_clock_link_arm_t))
    xQueueSend(s_acks, data, 0);
  else if (msg_id == HOSTED_AVB_MSG_CLOCK_LINK_EDGE &&
           length == sizeof(hosted_avb_clock_link_edge_t))
    xQueueSend(s_edges, data, 0);
}

/* The EMAC PTP clock at the latched edge: its system time read between
 * two timer latches, taken back to the edge. False while the clock is
 * being set or the read straddled a second. */
static bool ptp_at_edge(uint64_t edge_ticks, int64_t *ptp_ns) {
  for (int attempt = 0; attempt < 4; attempt++) {
    uint32_t control = EMAC_PTP.timestamp_ctrl.val;
    uint32_t seconds = EMAC_PTP.sys_seconds.ts_second;
    uint64_t before, after;
    uint32_t fraction;
    hosted_avb_timer_bracket(s_host_timer, &EMAC_PTP.sys_nanosec.val, &before,
                             &fraction, &after);
    bool digital = EMAC_PTP.timestamp_ctrl.ts_digit_bin_roll_ctrl;
    bool stable = EMAC_PTP.timestamp_ctrl.en_timestamp &&
                  !EMAC_PTP.timestamp_ctrl.ts_initialize &&
                  !EMAC_PTP.timestamp_ctrl.ts_update &&
                  seconds == EMAC_PTP.sys_seconds.ts_second &&
                  control == EMAC_PTP.timestamp_ctrl.val &&
                  after - before <= BRACKET_MAX_TICKS;
    fraction &= 0x7fffffffU;
    uint32_t nanos = digital
                         ? fraction
                         : (uint32_t)(((uint64_t)fraction * 1000000000ULL) >> 31);
    if (!stable || nanos >= 1000000000U)
      continue;
    uint64_t middle = before + (after - before) / 2;
    *ptp_ns = (int64_t)seconds * 1000000000LL + nanos -
              (int64_t)(middle - edge_ticks) * TICK_NS;
    return true;
  }
  return false;
}

/* Pulses the link and returns the host timer's latch of the edge. */
static bool emit_edge(uint64_t *edge_ticks) {
  uint64_t baseline = 0;
  gptimer_get_captured_count(s_host_timer, &baseline);
  gpio_set_level(HOST_GPIO, 1);
  esp_rom_delay_us(5);
  gpio_set_level(HOST_GPIO, 0);
  if (gptimer_get_captured_count(s_host_timer, edge_ticks) != ESP_OK)
    return false;
  return *edge_ticks != baseline;
}

/* Least-squares fit of the host PTP clock against the coprocessor timer
 * over pairs (oldest first), with the spread around it. */
static void fit_pairs(const hosted_avb_clock_pair_t *pairs, int count,
                      clock_fit_t *fit) {
  *fit = (clock_fit_t){0};
  if (count < 2)
    return;
  const hosted_avb_clock_pair_t *first = &pairs[0];
  double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
  for (int i = 0; i < count; i++) {
    double x = (double)(pairs[i].cp_ticks - first->cp_ticks) * TICK_NS;
    double y = (double)(pairs[i].host_ptp_ns - first->host_ptp_ns);
    sum_x += x;
    sum_y += y;
    sum_xx += x * x;
    sum_xy += x * y;
  }
  double slope = (count * sum_xy - sum_x * sum_y) /
                 (count * sum_xx - sum_x * sum_x);
  double intercept = (sum_y - slope * sum_x) / count;
  double sum_rr = 0, low = 0, high = 0;
  for (int i = 0; i < count; i++) {
    double x = (double)(pairs[i].cp_ticks - first->cp_ticks) * TICK_NS;
    double y = (double)(pairs[i].host_ptp_ns - first->host_ptp_ns);
    double residual = y - (intercept + slope * x);
    sum_rr += residual * residual;
    if (i == 0 || residual < low)
      low = residual;
    if (i == 0 || residual > high)
      high = residual;
  }
  *fit = (clock_fit_t){
      .valid = count >= FIT_MIN_PAIRS,
      .count = count,
      .anchor_ticks = first->cp_ticks,
      .anchor_host_ns = first->host_ptp_ns,
      .slope = slope,
      .intercept = intercept,
      .rms = sqrt(sum_rr / count),
      .peak_to_peak = high - low,
      .updated_us = esp_timer_get_time(),
  };
}

/* The fitted host time at a coprocessor timer reading. */
static double fit_host_ns(const clock_fit_t *fit, uint64_t cp_ticks) {
  double x = (double)(int64_t)(cp_ticks - fit->anchor_ticks) * TICK_NS;
  return (double)fit->anchor_host_ns + fit->intercept + fit->slope * x;
}

static void report_fit(const clock_fit_t *fit, uint32_t paired,
                       uint32_t missed, uint32_t steps) {
  if (fit->count < 4) {
    ESP_LOGW(TAG, "CLOCKLINK,%" PRIu32 ",%" PRIu32 ",too few pairs", paired,
             missed);
    return;
  }
  ESP_LOGI(TAG,
           "CLOCKLINK,%" PRIu32 ",%" PRIu32 ",rate %.3f ppm,residual rms %.1f "
           "ns p-p %.1f ns over %d edges,steps %" PRIu32,
           paired, missed, (fit->slope - 1.0) * 1e6, fit->rms,
           fit->peak_to_peak, fit->count, steps);
}

/* The ticks left before end, or 0 once it has passed. */
static TickType_t ticks_left(TickType_t end) {
  int32_t left = (int32_t)(end - xTaskGetTickCount());
  return left > 0 ? (TickType_t)left : 0;
}

static bool wait_ack(uint32_t sequence) {
  hosted_avb_clock_link_arm_t ack;
  TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(REPLY_TIMEOUT_MS);
  while (xQueueReceive(s_acks, &ack, ticks_left(end)) == pdTRUE) {
    if (ack.version == HOSTED_AVB_CLOCK_LINK_VERSION &&
        ack.host_boot == s_host_boot && ack.sequence == sequence)
      return ack.accepted;
  }
  return false;
}

static bool wait_edge(uint32_t sequence, hosted_avb_clock_link_edge_t *edge) {
  TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(REPLY_TIMEOUT_MS);
  while (xQueueReceive(s_edges, edge, ticks_left(end)) == pdTRUE) {
    if (edge->version == HOSTED_AVB_CLOCK_LINK_VERSION &&
        edge->host_boot == s_host_boot && edge->sequence == sequence)
      return edge->valid;
  }
  return false;
}

static void link_task(void *arg) {
  hosted_avb_clock_pair_t ring[FIT_SIZE];
  int ring_count = 0, ring_next = 0;
  uint32_t sequence = 0, paired = 0, missed = 0, steps = 0;
  clock_fit_t fit = {0};
  int64_t next_report = esp_timer_get_time() + REPORT_PERIOD_US;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(CONFIG_HOSTED_AVB_CLOCK_LINK_PERIOD_MS));
    if (esp_timer_get_time() >= next_report) {
      report_fit(&fit, paired, missed, steps);
      next_report = esp_timer_get_time() + REPORT_PERIOD_US;
    }
    sequence++;
    hosted_avb_clock_link_arm_t arm = {
        .version = HOSTED_AVB_CLOCK_LINK_VERSION,
        .host_boot = s_host_boot,
        .sequence = sequence,
    };
    xQueueReset(s_acks);
    xQueueReset(s_edges);
    uint64_t edge_ticks;
    int64_t ptp_ns;
    hosted_avb_clock_link_edge_t edge;
    if (hosted_avb_peer_send(HOSTED_AVB_MSG_CLOCK_LINK_ARM,
                             (const uint8_t *)&arm, sizeof(arm)) != ESP_OK ||
        !wait_ack(sequence) || !emit_edge(&edge_ticks) ||
        !ptp_at_edge(edge_ticks, &ptp_ns) || !wait_edge(sequence, &edge)) {
      missed++;
      continue;
    }
    hosted_avb_clock_pair_t pair = {
        .sequence = sequence,
        .host_ptp_ns = ptp_ns,
        .cp_ticks = edge.edge_ticks,
        .cp_mac_ticks = edge.mac_before_ticks +
                        (edge.mac_after_ticks - edge.mac_before_ticks) / 2,
        .cp_mac_us = edge.mac_us,
    };
    /* A restarted coprocessor has a new timer; a pair far off the fit
     * means the host clock was stepped. Either way, fit again from here. */
    if (fit.count && edge.coprocessor_boot != fit.coprocessor_boot) {
      ring_count = 0;
      ESP_LOGW(TAG, "Coprocessor restarted: fitting again");
    } else if (fit.count >= 2 &&
               fabs((double)ptp_ns - fit_host_ns(&fit, pair.cp_ticks)) >
                   STEP_LIMIT_NS) {
      ring_count = 0;
      steps++;
      ESP_LOGW(TAG, "Clock step of %.0f ns: fitting again",
               (double)ptp_ns - fit_host_ns(&fit, pair.cp_ticks));
    }
    ring[ring_next] = pair;
    ring_next = (ring_next + 1) % FIT_SIZE;
    if (ring_count < FIT_SIZE)
      ring_count++;
    paired++;
    hosted_avb_clock_pair_t ordered[FIT_SIZE];
    for (int i = 0; i < ring_count; i++)
      ordered[i] = ring[(ring_next - ring_count + i + FIT_SIZE) % FIT_SIZE];
    fit_pairs(ordered, ring_count, &fit);
    fit.coprocessor_boot = edge.coprocessor_boot;
    portENTER_CRITICAL(&s_latest_lock);
    s_latest = pair;
    s_have_latest = true;
    s_fit = fit;
    portEXIT_CRITICAL(&s_latest_lock);
  }
}

esp_err_t hosted_avb_clock_link_start(void) {
  if (s_link_task)
    return ESP_OK;
  s_host_boot = esp_random() | 1;
  gpio_config_t pin = {
      .pin_bit_mask = 1ULL << HOST_GPIO,
      .mode = GPIO_MODE_INPUT_OUTPUT, /* the timer latches our own edge */
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_set_level(HOST_GPIO, 0);
  ESP_RETURN_ON_ERROR(gpio_config(&pin), TAG, "GPIO");
  ESP_RETURN_ON_ERROR(new_edge_timer(HOST_GPIO, &s_host_timer), TAG,
                      "edge timer");
  s_acks = xQueueCreate(4, sizeof(hosted_avb_clock_link_arm_t));
  s_edges = xQueueCreate(4, sizeof(hosted_avb_clock_link_edge_t));
  if (!s_acks || !s_edges)
    return ESP_ERR_NO_MEM;
  ESP_RETURN_ON_ERROR(hosted_avb_peer_register(
                          HOSTED_AVB_MSG_CLOCK_LINK_ARM_ACK, on_reply, NULL),
                      TAG, "register ARM_ACK");
  ESP_RETURN_ON_ERROR(hosted_avb_peer_register(HOSTED_AVB_MSG_CLOCK_LINK_EDGE,
                                               on_reply, NULL),
                      TAG, "register EDGE");
  /* Core 0: unpinned, it sat behind an endpoint talker's busy-waiting
   * task on core 1 and paired no edges while a stream played, so the
   * host's Wi-Fi time drifted (650 us over 150 s). */
  if (xTaskCreatePinnedToCore(link_task, "clock_link", 4096, NULL, 5,
                              &s_link_task, 0) != pdPASS)
    return ESP_ERR_NO_MEM;
  ESP_LOGI(TAG, "Clock link on GPIO%d, an edge every %d ms", HOST_GPIO,
           CONFIG_HOSTED_AVB_CLOCK_LINK_PERIOD_MS);
  return ESP_OK;
}

bool hosted_avb_clock_link_latest(hosted_avb_clock_pair_t *pair) {
  portENTER_CRITICAL(&s_latest_lock);
  bool have = s_have_latest;
  if (have)
    *pair = s_latest;
  portEXIT_CRITICAL(&s_latest_lock);
  return have;
}

bool hosted_avb_clock_link_fit(int64_t *cp_ns, int64_t *host_ns,
                               double *rate, uint32_t *coprocessor_boot) {
  portENTER_CRITICAL(&s_latest_lock);
  clock_fit_t fit = s_fit;
  uint64_t latest_ticks = s_latest.cp_ticks;
  portEXIT_CRITICAL(&s_latest_lock);
  if (!fit.valid || esp_timer_get_time() - fit.updated_us > FIT_MAX_AGE_US)
    return false;
  *cp_ns = (int64_t)latest_ticks * TICK_NS;
  *host_ns = (int64_t)llround(fit_host_ns(&fit, latest_ticks));
  *rate = fit.slope;
  *coprocessor_boot = fit.coprocessor_boot;
  return true;
}

bool hosted_avb_clock_link_to_host(int64_t cp_ns, int64_t *host_ns) {
  portENTER_CRITICAL(&s_latest_lock);
  clock_fit_t fit = s_fit;
  portEXIT_CRITICAL(&s_latest_lock);
  if (!fit.valid || esp_timer_get_time() - fit.updated_us > FIT_MAX_AGE_US)
    return false;
  int64_t span = cp_ns - (int64_t)fit.anchor_ticks * TICK_NS;
  if (span < -FIT_MAX_SPAN_NS || span > FIT_MAX_SPAN_NS)
    return false;
  *host_ns = fit.anchor_host_ns +
             (int64_t)llround(fit.intercept + fit.slope * (double)span);
  return true;
}

#endif /* host */

/* --------------------------------------------------------- coprocessor */
#if CONFIG_HOSTED_AVB_BUILD_COPROCESSOR_HANDLER

#define CP_GPIO CONFIG_HOSTED_AVB_CLOCK_LINK_COPROCESSOR_GPIO
#if CONFIG_IDF_TARGET_ESP32C6
#include "esp_wifi.h"
/* The Wi-Fi MAC's microsecond clock, read beside each edge. */
#define MAC_CLOCK_REG ((volatile const uint32_t *)0x600ad000U)
#endif

static DRAM_ATTR gptimer_handle_t s_cp_timer;
static DRAM_ATTR portMUX_TYPE s_cp_lock = portMUX_INITIALIZER_UNLOCKED;
static DRAM_ATTR struct {
  bool armed;
  uint32_t host_boot;
  uint32_t sequence;
  uint64_t baseline;
} s_arm;
static DRAM_ATTR hosted_avb_clock_link_edge_t s_edge;
static DRAM_ATTR TaskHandle_t s_cp_task;
static uint32_t s_cp_boot;
/* The last soft latch taken outside the edge handler. A bracketed read
 * that straddles an edge overwrites the edge's hardware latch with it. */
static DRAM_ATTR volatile uint64_t s_last_soft_latch;

static void IRAM_ATTR on_edge(void *arg) {
  uint64_t ticks;
  /* The hardware latch first: the bracket's soft latches overwrite it. */
  if (gptimer_get_captured_count(s_cp_timer, &ticks) != ESP_OK)
    return;
  bool overwritten = ticks == s_last_soft_latch;
  uint64_t before = 0, after = 0;
  uint32_t mac_us = 0;
#ifdef MAC_CLOCK_REG
  hosted_avb_timer_bracket(s_cp_timer, MAC_CLOCK_REG, &before, &mac_us,
                           &after);
#endif
  bool notify = false;
  portENTER_CRITICAL_ISR(&s_cp_lock);
  if (s_arm.armed && ticks != s_arm.baseline) {
    s_edge = (hosted_avb_clock_link_edge_t){
        .version = HOSTED_AVB_CLOCK_LINK_VERSION,
        .valid = !overwritten,
        .host_boot = s_arm.host_boot,
        .sequence = s_arm.sequence,
        .coprocessor_boot = s_cp_boot,
        .edge_ticks = ticks,
        .mac_before_ticks = before,
        .mac_after_ticks = after,
        .mac_us = mac_us,
    };
    s_arm.armed = false;
    notify = true;
  }
  portEXIT_CRITICAL_ISR(&s_cp_lock);
  if (notify) {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_cp_task, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

uint32_t hosted_avb_clock_link_coprocessor_boot(void) { return s_cp_boot; }

bool hosted_avb_clock_link_coprocessor_read(volatile const uint32_t *reg,
                                            int64_t *before_ns, uint32_t *value,
                                            int64_t *after_ns) {
  if (!s_cp_timer)
    return false;
  uint64_t before, after;
  hosted_avb_timer_bracket(s_cp_timer, reg, &before, value, &after);
  s_last_soft_latch = after;
  *before_ns = (int64_t)before * TICK_NS;
  *after_ns = (int64_t)after * TICK_NS;
  return true;
}

#ifdef MAC_CLOCK_REG
#define MAP_SAMPLES 256
#define MAP_MAX_BRACKET_NS 1000

/* Each bracketed read bounds the map; at random phases against the MAC
 * clock's microsecond steps the bounds close in to about the bracket. */
bool hosted_avb_clock_link_map_mac(hosted_avb_mac_map_t *map) {
  *map = (hosted_avb_mac_map_t){0};
  uint32_t phase = esp_random() | 1;
  uint32_t anchor_mac = 0;
  int64_t lower = INT64_MIN, upper = INT64_MAX;
  unsigned used = 0;
  for (unsigned attempt = 0; attempt < MAP_SAMPLES; attempt++) {
    phase ^= phase << 13;
    phase ^= phase >> 17;
    phase ^= phase << 5;
    for (unsigned delay = phase & 31; delay; --delay)
      __asm__ volatile("nop");
    int64_t before, after;
    uint32_t mac;
    if (!hosted_avb_clock_link_coprocessor_read(MAC_CLOCK_REG, &before, &mac,
                                                &after))
      return false;
    if (after - before > MAP_MAX_BRACKET_NS)
      continue;
    if (!used)
      anchor_mac = mac;
    int32_t delta = (int32_t)(mac - anchor_mac);
    if (delta < 0 || delta > 1000000)
      continue;
    /* The timer at MAC time anchor_mac lies between these. */
    int64_t low = before - (int64_t)delta * 1000 - 1000;
    int64_t high = after + TICK_NS - (int64_t)delta * 1000;
    if (low > lower)
      lower = low;
    if (high < upper)
      upper = high;
    used++;
  }
  if (used < MAP_SAMPLES / 4 || lower > upper || upper - lower > 1000)
    return false;
  *map = (hosted_avb_mac_map_t){
      .valid = true,
      .mac_us = anchor_mac,
      .cp_ns = lower + (upper - lower) / 2,
      .uncertainty_ns = (uint32_t)((upper - lower + 1) / 2),
  };
  return true;
}
#define TSF_TRIES 8
#define TSF_MAX_BRACKET_NS 50000

/* The TSF is a microsecond count read through the Wi-Fi library, so its
 * read is bracketed by two latches rather than inside one: it reached
 * the value read within a microsecond before the read, which lies
 * between the latches. */
bool hosted_avb_clock_link_ap_tsf(int64_t *tsf_us, int64_t *cp_ns,
                                  uint32_t *uncertainty_ns) {
  int64_t best_low = 0, best_high = 0, best_tsf = 0;
  bool found = false;
  for (unsigned attempt = 0; attempt < TSF_TRIES; attempt++) {
    int64_t before, start, end, after;
    uint32_t unused;
    if (!hosted_avb_clock_link_coprocessor_read(MAC_CLOCK_REG, &before,
                                                &unused, &start))
      return false;
    int64_t tsf = esp_wifi_get_tsf_time(WIFI_IF_AP);
    if (!hosted_avb_clock_link_coprocessor_read(MAC_CLOCK_REG, &end, &unused,
                                                &after))
      return false;
    if (tsf <= 0)
      return false;
    if (end - start > TSF_MAX_BRACKET_NS)
      continue;
    int64_t low = start - 1000, high = end;
    if (!found || high - low < best_high - best_low) {
      best_low = low;
      best_high = high;
      best_tsf = tsf;
      found = true;
    }
  }
  if (!found)
    return false;
  *tsf_us = best_tsf;
  *cp_ns = best_low + (best_high - best_low) / 2;
  *uncertainty_ns = (uint32_t)((best_high - best_low + 1) / 2);
  return true;
}
#else
bool hosted_avb_clock_link_map_mac(hosted_avb_mac_map_t *map) {
  *map = (hosted_avb_mac_map_t){0};
  return false;
}

bool hosted_avb_clock_link_ap_tsf(int64_t *tsf_us, int64_t *cp_ns,
                                  uint32_t *uncertainty_ns) {
  return false;
}
#endif

static void on_arm(uint32_t msg_id, const uint8_t *data, size_t length,
                   void *user) {
  hosted_avb_clock_link_arm_t arm;
  if (msg_id != HOSTED_AVB_MSG_CLOCK_LINK_ARM || length != sizeof(arm))
    return;
  memcpy(&arm, data, sizeof(arm));
  if (arm.version != HOSTED_AVB_CLOCK_LINK_VERSION)
    return;
  uint64_t baseline = 0;
  arm.accepted =
      gptimer_get_captured_count(s_cp_timer, &baseline) == ESP_OK;
  portENTER_CRITICAL(&s_cp_lock);
  s_arm.armed = arm.accepted;
  s_arm.host_boot = arm.host_boot;
  s_arm.sequence = arm.sequence;
  s_arm.baseline = baseline;
  portEXIT_CRITICAL(&s_cp_lock);
  arm.coprocessor_boot = s_cp_boot;
  hosted_avb_peer_send(HOSTED_AVB_MSG_CLOCK_LINK_ARM_ACK,
                       (const uint8_t *)&arm, sizeof(arm));
}

static void cp_task(void *arg) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    hosted_avb_clock_link_edge_t edge;
    portENTER_CRITICAL(&s_cp_lock);
    edge = s_edge;
    portEXIT_CRITICAL(&s_cp_lock);
    hosted_avb_peer_send(HOSTED_AVB_MSG_CLOCK_LINK_EDGE,
                         (const uint8_t *)&edge, sizeof(edge));
  }
}

esp_err_t hosted_avb_clock_link_coprocessor_init(void) {
  s_cp_boot = esp_random() | 1;
  gpio_config_t pin = {
      .pin_bit_mask = 1ULL << CP_GPIO,
      .mode = GPIO_MODE_INPUT,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_RETURN_ON_ERROR(gpio_config(&pin), TAG, "GPIO");
  ESP_RETURN_ON_ERROR(new_edge_timer(CP_GPIO, &s_cp_timer), TAG,
                      "edge timer");
  if (xTaskCreate(cp_task, "clock_link", 3072, NULL, 5, &s_cp_task) != pdPASS)
    return ESP_ERR_NO_MEM;
  esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    return err;
  ESP_RETURN_ON_ERROR(gpio_isr_handler_add(CP_GPIO, on_edge, NULL), TAG,
                      "ISR");
  ESP_RETURN_ON_ERROR(gpio_set_intr_type(CP_GPIO, GPIO_INTR_POSEDGE), TAG,
                      "interrupt type");
  ESP_RETURN_ON_ERROR(gpio_intr_enable(CP_GPIO), TAG, "interrupt");
  ESP_RETURN_ON_ERROR(
      hosted_avb_peer_register(HOSTED_AVB_MSG_CLOCK_LINK_ARM, on_arm, NULL),
      TAG, "register ARM");
  ESP_LOGI(TAG, "Clock link on GPIO%d", CP_GPIO);
  return ESP_OK;
}

#endif /* coprocessor */

#endif /* CONFIG_HOSTED_AVB_CLOCK_LINK */
