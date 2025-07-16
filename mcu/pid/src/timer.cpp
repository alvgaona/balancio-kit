#include "timer.h"
#include "config.h"
#include <esp32-hal-timer.h>

// ISR at 1/LOOP_PERIOD Hz
void IRAM_ATTR onTime() { controlFlag = true; }

void timerInit(void) {
  // Configure the Prescaler at 80. Clock frequency at 80Mhz.
  // 80000000 / 80 = 1000000 ticks / second
  hw_timer_t *timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTime, true);

  // Sets timer at 100Hz.
  int scaler = 1000000 * LOOP_PERIOD;
  timerAlarmWrite(timer, scaler, true);
  timerAlarmEnable(timer);
}
