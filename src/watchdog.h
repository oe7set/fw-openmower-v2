//
// Software watchdog for the OpenMower V2 firmware.
//

#ifndef WATCHDOG_H
#define WATCHDOG_H
#ifdef __cplusplus
extern "C" {
#endif

// Number of milliseconds the idle thread may be starved before the watchdog
// resets the board. The idle thread runs at the lowest priority, so if it has
// not executed for this long the CPU is genuinely stuck (a runaway loop or a
// priority-inversion deadlock holding every higher-priority thread). This is
// deliberately generous so that normal bursty load never trips it; the real
// hang sources it backstops (e.g. a blocking UART/socket call) are also fixed
// at the source.
#define WATCHDOG_TIMEOUT_MS 8000

// Initializes and arms the software watchdog. Call once after the services have
// been started. A reset is performed via NVIC_SystemReset(), which goes through
// the normal bootloader path -- unlike the hardware IWDG, this cannot cause a
// boot loop because nothing persists across the reset.
void InitWatchdog(void);

#ifdef __cplusplus
}
#endif
#endif  // WATCHDOG_H
