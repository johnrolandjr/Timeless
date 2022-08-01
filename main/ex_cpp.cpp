#include <Arduino.h>
#include "ex_cpp.hh"

#include <TimerInterrupt.h>
#include <TimerInterrupt.hpp>
#include <ISR_Timer.h>
#include <ISR_Timer.hpp>

uint32_t events_g;
bool bStarted_g;

void process_event(void)
{
	uint32_t evt;

  // Begin critical section to safely capture the shared variable
  noInterrupts();
  evt = events_g;
  events_g = 0;
  interrupts();

  // Process all of the events triggered
  if ((evt & TIMER_1_ISR_EVT) != NOT_TRIGGERED)
  {
    // Every time timer 1 occurs, perform the main operation
    process_main_loop();
  }
}

void process_main_loop(void)
{
  debounce_input_pins();
  if (bStarted_g)
  {
    // Currently animating. Check to see whether it's time to end now.
    Serial.println("A");
    if (showtime_expired() || !magnet_detected())
    {
      stop_animation();
    }
  }
  else
  {
    // Currently not animating. Check to see whether it's time to start now.
    Serial.println("X");
    if (magnet_detected() || (duration_sw_held() > BACKUP_START_SW_HELD_ITERATION))
    {
      start_animation();
    }
  }
}

void start_animation(void)
{
  Serial.println("Start Animation");
  bStarted_g = true;
}

void stop_animation(void)
{
  Serial.println("Stop Animation");
  bStarted_g = false;
}

void timer_1_handler(void)
{
  // Timer ISR
  events_g |= TIMER_1_ISR_EVT;
}

void debounce_input_pins(void)
{
  
}

bool magnet_detected(void)
{
  // TODO implement once debounce_input_pins is implemented
  return false;
}

bool switch_pressed(void)
{
  // TODO implement once debounce_input_pins is implemented
  return false;
}
uint32_t duration_sw_held(void)
{
  // Todo implemente
  return 0;
}
bool showtime_expired(void)
{
  // TODO implement
  return true;
}

void init_state(void)
{
  bStarted_g = false;
}

void init_timers(void)
{
  events_g = NO_PENDING_EVENTS;
  
  ITimer1.init();
  // Interval in unsigned long millisecs
  if (ITimer1.attachInterruptInterval(TIMER_1_INTERVAL_MS, timer_1_handler))
    Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or timer");
}
