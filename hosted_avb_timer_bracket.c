/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Built into the GPTimer driver's library (see CMakeLists.txt) so it can
 * use the driver's private handle: latches the timer, reads one 32-bit
 * register, latches the timer again, all under the driver's own lock, so
 * the two readings bracket the register read tightly.
 */
#include <stdint.h>

#include "esp_attr.h"
#include "gptimer_priv.h"
#include "hal/timer_ll.h"

static inline __attribute__((always_inline)) uint64_t
latch(timer_hal_context_t *timer) {
  timer_ll_trigger_soft_capture(timer->dev, timer->timer_id);
  return timer_ll_get_counter_value(timer->dev, timer->timer_id);
}

void IRAM_ATTR hosted_avb_timer_bracket(gptimer_handle_t timer,
                                        volatile const uint32_t *reg,
                                        uint64_t *before, uint32_t *value,
                                        uint64_t *after) {
  portENTER_CRITICAL_SAFE(&timer->spinlock);
  uint64_t first = latch(&timer->hal);
  uint32_t sample = *reg;
  uint64_t second = latch(&timer->hal);
  portEXIT_CRITICAL_SAFE(&timer->spinlock);
  *before = first;
  *value = sample;
  *after = second;
}
