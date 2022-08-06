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
    #if defined(DISABLE_BACKUP_SWITCH)
      if (magnet_detected())
    #else
      if (magnet_detected() || (duration_sw_held() > BACKUP_SW_HELD_DURATION))
    #endif // DISABLE_BACKUP_SWITCH
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

  // Turn on PWM Led
  analogWrite(PWM_LED_PIN, 128);
  analogWrite(PWM_MAG_PIN, 128);

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

  // Turn on PWMs (LED and MAGNET)
  analogWrite(PWM_LED_PIN, 0);
  analogWrite(PWM_MAG_PIN, 0);
  
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
  int state = digitalRead(MAG_DETECT_PIN);
  
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
  pinMode(MAG_DETECT_PIN, INPUT);

  // Initialize backup switch as digital input
  pinMode(BK_UP_PIN, INPUT);

  // Initialize led pwm pin as digital output
  pinMode(PWM_LED_PIN, OUTPUT);

  // Initialize mag pwm pin as digital output
  pinMode(PWM_MAG_PIN, OUTPUT);

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

void print_timer_0_cfg(void)
{
  my_printf("Timer 0");
  my_printf("----------------------------------- ");
  my_printf("TCCR0A = 0x" + String(TCCR0A, HEX));
  my_printf("TCCR0B = 0x" + String(TCCR0B, HEX));
  my_printf("TCNT0 = 0x" + String(TCNT0, HEX));
  my_printf("OCR0A = 0x" + String(OCR0A, HEX));
  my_printf("OCR0B = 0x" + String(OCR0B, HEX));
  my_printf("TIMSK0 = 0x" + String(TIMSK0, HEX));
  my_printf("TIFR0 = 0x" + String(TIFR0, HEX));
  my_printf("----------------------------------- ");
}

void print_timer_1_cfg(void)
{
  uint16_t h;
  uint16_t l;
  uint16_t u16val;

  my_printf("Timer 1 (most likely used by delay()");
  my_printf("----------------------------------- ");
  my_printf("TCCR1A = 0x" + String(TCCR1A, HEX));
  my_printf("TCCR1B = 0x" + String(TCCR1B, HEX));
  my_printf("TCCR1C = 0x" + String(TCCR1C, HEX));
  h = TCNT1H;
  l = TCNT1L;
  u16val = (h << 8) + l;
  my_printf("TCNT1 = 0x" + String(u16val, HEX));
  h = OCR1AH;
  l = OCR1AL;
  u16val = (h << 8) + l;
  my_printf("OCR1A = 0x" + String(u16val, HEX));
  h = OCR1BH;
  l = OCR1BL;
  u16val = (h << 8) + l;
  my_printf("OCR1B = 0x" + String(u16val, HEX));
  my_printf("TIMSK1 = 0x" + String(TIMSK1, HEX));
  my_printf("TIFR1 = 0x" + String(TIFR1, HEX));
  my_printf("----------------------------------- ");
}

void print_timer_2_cfg(void)
{
  my_printf("Timer 2");
  my_printf("----------------------------------- ");
  my_printf("TCCR2A = 0x" + String(TCCR2A, HEX));
  my_printf("TCCR2B = 0x" + String(TCCR2B, HEX));
  my_printf("TCNT2 = 0x" + String(TCNT2, HEX));
  my_printf("OCR2A = 0x" + String(OCR2A, HEX));
  my_printf("OCR2B = 0x" + String(OCR2B, HEX));
  my_printf("TIMSK2 = 0x" + String(TIMSK2, HEX));
  my_printf("TIFR2 = 0x" + String(TIFR2, HEX));
  my_printf("ASSR = 0x" + String(ASSR, HEX));
  my_printf("GTCCR = 0x" + String(GTCCR, HEX));
  my_printf("----------------------------------- ");
}

void print_timer_cfg(void)
{
  print_timer_0_cfg();
  //print_timer_1_cfg();
  //print_timer_2_cfg();

  // Modify the mode for time 0, and readback to verify
  // Updating WGM 3 -> 7
  TCCR0B |= bit(3);
  // Updating OCRA to 0xFF (to ensure same frequency)
  OCR0A = 0xFF;
  
  //Readback Verify
  print_timer_0_cfg();
}