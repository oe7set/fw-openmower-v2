//
// Created by clemens on 29.09.24.
//

#include "heartbeat.h"

#include "ch.h"
#include "hal.h"

// Gets updated by the idle task to the current tick value.
volatile uint32_t last_idle_tick = 0;

static volatile uint8_t blink_state = 0;

static virtual_timer_t heartbeat_timer;

static void heartbeat_timer_cb(struct ch_virtual_timer *tp, void *arg) {
  (void)arg;
  (void)tp;
  bool elapsed = TIME_I2MS(chVTTimeElapsedSinceX(last_idle_tick)) > HEARTBEAT_RECENT_MS;

#if HAL_USE_WDG
  // Feed the independent watchdog, but ONLY once it has been started (after boot
  // completes in main()) AND while the idle thread is still being scheduled
  // (i.e. the system is alive and not CPU-bound deadlocked). If a thread spins
  // or deadlocks at >= NORMALPRIO, the idle thread stops running, last_idle_tick
  // goes stale, we stop feeding, and the IWDG resets the board after its timeout
  // — turning a permanent hang (previously only recoverable by a power-cycle)
  // into an automatic reboot. The WDG_READY guard makes the callback a no-op
  // during boot before wdgStart(); the *I form is required in this virtual-timer
  // (locked ISR) context.
  if (!elapsed && WDGD1.state == WDG_READY) {
    wdgResetI(&WDGD1);
  }
#endif
  switch (blink_state) {
    case 0:
      if (elapsed) {
        palClearLine(LINE_HEARTBEAT_LED_RED);
        palSetLine(LINE_HEARTBEAT_LED_GREEN);
      } else {
        palClearLine(LINE_HEARTBEAT_LED_GREEN);
        palSetLine(LINE_HEARTBEAT_LED_RED);
      }
      chSysLockFromISR();
      chVTSetI(&heartbeat_timer, TIME_MS2I(100), heartbeat_timer_cb, NULL);
      chSysUnlockFromISR();
      blink_state++;
      break;
    case 1:
      palSetLine(LINE_HEARTBEAT_LED_GREEN);
      palSetLine(LINE_HEARTBEAT_LED_RED);
      chSysLockFromISR();
      chVTSetI(&heartbeat_timer, TIME_MS2I(100), heartbeat_timer_cb, NULL);
      chSysUnlockFromISR();
      blink_state++;
      break;
    case 2:
      if (elapsed) {
        palClearLine(LINE_HEARTBEAT_LED_RED);
        palSetLine(LINE_HEARTBEAT_LED_GREEN);
      } else {
        palClearLine(LINE_HEARTBEAT_LED_GREEN);
        palSetLine(LINE_HEARTBEAT_LED_RED);
      }
      chSysLockFromISR();
      chVTSetI(&heartbeat_timer, TIME_MS2I(100), heartbeat_timer_cb, NULL);
      chSysUnlockFromISR();
      blink_state++;
      break;
    default:
      palSetLine(LINE_HEARTBEAT_LED_GREEN);
      palSetLine(LINE_HEARTBEAT_LED_RED);

      chSysLockFromISR();
      chVTSetI(&heartbeat_timer, TIME_MS2I(500), heartbeat_timer_cb, NULL);
      chSysUnlockFromISR();
      blink_state = 0;
      break;
  }
}

void InitHeartbeat() {
  chVTObjectInit(&heartbeat_timer);
  chSysLock();
  chVTSetI(&heartbeat_timer, TIME_MS2I(100), heartbeat_timer_cb, NULL);
  chSysUnlock();
}
