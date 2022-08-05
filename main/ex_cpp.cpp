#include <Arduino.h>
#include "ex_cpp.hh"
#include "pins.hh"

#include <TimerInterrupt.h>
#include <TimerInterrupt.hpp>
#include <ISR_Timer.h>
#include <ISR_Timer.hpp>

uint32_t events_g;
bool bStarted_g;
uint32_t backup_sw_cnt_g;
int32_t showtime_count_g;

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
  if (bStarted_g)
  {
    // Currently animating. Check to see whether it's time to end now.
    if (showtime_expired())
    {
      stop_animation();
    }
  }
  else
  {
    // Currently not animating. Check to see whether it's time to start now.
    if (magnet_detected() || (duration_sw_held() > BACKUP_SW_HELD_DURATION))
    {
      start_animation();
    }
  }
}

void start_animation(void)
{
  my_printf("Start Animation");

  bStarted_g = true;
  showtime_count_g = SHOWTIME_DURATION;

#if defined(DEBUG)
  // As a visual que, blink led 1 time
  digitalWrite(BRD_LED_PIN, HIGH);
  delay(100);
  digitalWrite(BRD_LED_PIN, LOW);
#endif // DEBUG
}

void stop_animation(void)
{
  my_printf("Stop Animation");
  bStarted_g = false;
  backup_sw_cnt_g = 0;
#if defined(DEBUG)
  // As a visual que, blink led 3 times
  for (int i=0; i<3; i++)
  {
    digitalWrite(BRD_LED_PIN, HIGH);
    delay(100);
    digitalWrite(BRD_LED_PIN, LOW);
    delay(200);
  }
#endif // DEBUG
}

void timer_1_handler(void)
{
  // Timer ISR
  events_g |= TIMER_1_ISR_EVT;
}

bool magnet_detected(void)
{
  int state = digitalRead(MAG_ACT_PIN);
  
  return (state == MAG_STATE_DETECTED);
}

uint32_t duration_sw_held(void)
{
  int state = digitalRead(BK_UP_PIN);
  backup_sw_cnt_g++;

  if (state == BACKUP_SW_STATE_RELEASED)
    backup_sw_cnt_g = 0;

  return backup_sw_cnt_g;
}

bool showtime_expired(void)
{
  bool bExpired = false;
  if (--showtime_count_g <= 0)
    bExpired = true;
  return bExpired;
}

void init_serial(void)
{
#if defined(DEBUG)
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
#endif // DEBUG
}

void init_state(void)
{
  bStarted_g = false;
}

void init_pins(void)
{
  // Initialize magnet pin as digital input
  pinMode(MAG_ACT_PIN, INPUT);

  // Initialize backup switch as digital input
  pinMode(BK_UP_PIN, INPUT);

#if defined(DEBUG)
  // Initialize Board LED for debug
  pinMode(BRD_LED_PIN, OUTPUT);

  // As a visual que to know that we have started, toggle the pin 3 times
  digitalWrite(BRD_LED_PIN, LOW);
  for (int i=0; i<3; i++)
  {
    digitalWrite(BRD_LED_PIN, HIGH);
    delay(100);
    digitalWrite(BRD_LED_PIN, LOW);
    delay(200);
  }
#endif //DEBUG
}



void init_timers(void)
{
  events_g = NO_PENDING_EVENTS;
  
  ITimer1.init();

  // Interval in unsigned long millisecs
#if defined(DEBUG)
  if (ITimer1.attachInterruptInterval(TIMER_1_INTERVAL_MS, timer_1_handler))
    my_printf("Starting  ITimer OK, millis() = " + String(millis()));
  else
    my_printf("Can't set ITimer. Select another freq. or timer");
#else
  ITimer1.attachInterruptInterval(TIMER_1_INTERVAL_MS, timer_1_handler);
#endif // DEBUG
}
