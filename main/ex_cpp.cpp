#include <Arduino.h>
#include "ex_cpp.hh"

#include <TimerInterrupt.h>
#include <TimerInterrupt.hpp>
#include <ISR_Timer.h>
#include <ISR_Timer.hpp>

uint32_t events_g;

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
  Serial.println("Main Event");
}

void timer_1_handler(void)
{
  // Timer ISR
  events_g |= TIMER_1_ISR_EVT;
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
