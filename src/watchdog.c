//
// Software watchdog for the OpenMower V2 firmware.
//

#include "watchdog.h"

#include "ch.h"
#include "hal.h"

// Updated by the ChibiOS idle-loop hook (see CH_CFG_IDLE_LOOP_HOOK in
// cfg/chconf.h). Reflects the last time the lowest-priority idle thread ran.
extern volatile uint32_t last_idle_tick;

static virtual_timer_t watchdog_timer;

// How often the watchdog checks the idle liveness, in milliseconds. Much
// shorter than WATCHDOG_TIMEOUT_MS so the reset fires promptly once the
// threshold is crossed.
#define WATCHDOG_CHECK_INTERVAL_MS 500

static void watchdog_timer_cb(struct ch_virtual_timer *tp, void *arg) {
  (void)tp;
  (void)arg;

  // If the idle thread has not run for longer than the timeout, the CPU is
  // starved: a higher-priority thread is spinning and never yielding (runaway
  // loop, livelock). Reset to recover. Note this does NOT catch a thread that
  // is *blocked* on a wait (mutex/UART/socket) -- in that case the idle thread
  // still runs. Those blocking hangs are fixed at their source (bounded UART
  // and socket sends); this watchdog is the last-resort backstop for an
  // unexpected spin.
  if (TIME_I2MS(chVTTimeElapsedSinceX(last_idle_tick)) > WATCHDOG_TIMEOUT_MS) {
    NVIC_SystemReset();
    // Not reached.
  }

  chSysLockFromISR();
  chVTSetI(&watchdog_timer, TIME_MS2I(WATCHDOG_CHECK_INTERVAL_MS), watchdog_timer_cb, NULL);
  chSysUnlockFromISR();
}

void InitWatchdog(void) {
  // Seed the idle timestamp to "now" so the initial check has a sane baseline
  // even if the idle thread has not been scheduled yet.
  last_idle_tick = chVTGetSystemTime();

  chVTObjectInit(&watchdog_timer);
  chSysLock();
  chVTSetI(&watchdog_timer, TIME_MS2I(WATCHDOG_CHECK_INTERVAL_MS), watchdog_timer_cb, NULL);
  chSysUnlock();
}
