#include <Arduino.h>
#include "ex_cpp.hh"
#include "pins.hh"

#include <TimerInterrupt.h>
#include <TimerInterrupt.hpp>
#include <ISR_Timer.h>
#include <ISR_Timer.hpp>

uint32_t events_g;
bool bStarted_g;
uint8_t led_tick_period_s;
uint8_t mag_tick_period_s;
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
  int32_t period_ticks, period_shift, period_delta;
  my_printf("Start Animation");

  bStarted_g = true;
  showtime_count_g = SHOWTIME_DURATION;

  // In the future, we may read an analog voltage and change the delta periods between magnet and led.
  // CLKsys (16Mhz) prescaled(16) = 1MHz. CLKio (1MHz) prescaled(64) = 15625 Hz --> Range of (61Hz - 15625Hz)
  period_ticks = 195; // 195 measured 79.5Hz
  period_shift = 0; // (~0.3Hz / tick)

  int delta_pot_val = analogRead(DELTA_POT_PIN);
  period_delta = get_delta(delta_pot_val); // (~0.3Hz / tick)

  // Update the frequencies prior to turning them on
  uint32_t led_tick_period = period_ticks + period_shift;
  uint32_t mag_tick_period = period_ticks + period_shift + period_delta;
  update_led_freq(led_tick_period);
  update_mag_freq(mag_tick_period);

  // Turn on PWM Led
  // In the future, we may want to be able to change the brightness (aka the duty cycle)
  int brightness_pot_val = analogRead(BRIGHTNESS_POT_PIN);
  float duty = get_brightness(brightness_pot_val);
  analogWrite(PWM_LED_PIN, (uint32_t)(led_tick_period_s * duty));
  analogWrite(PWM_MAG_PIN, (uint32_t)(mag_tick_period_s * duty));

  #if defined(DEBUG)
    // As a visual que, blink led 1 time
    blink_board_led(1);
  #endif // DEBUG
}

void stop_animation(void)
{
  my_printf("Stop Animation");
  bStarted_g = false;
  backup_sw_cnt_g = 0;

  // Turn off PWMs (LED and MAGNET)
  analogWrite(PWM_LED_PIN, 0);
  analogWrite(PWM_MAG_PIN, 0);
  
  #if defined(DEBUG)
    // As a visual que, blink led 3 times
    blink_board_led(3);
  #endif // DEBUG
}

uint32_t get_delta(int pot_val)
{
  uint32_t delta = 0; // (~0.3Hz / tick)
  float percent = 0.0f;
  if (pot_val > ANA_MAX_3V3_READ)
  {
    pot_val = ANA_MAX_3V3_READ;
  }
  percent = ((float)pot_val / ANA_MAX_3V3_READ);

  return ((uint32_t)(MAX_DELTA * percent));
}

float get_brightness(int pot_val)
{
  float duty = 0.0f;
  if (pot_val > ANA_MAX_3V3_READ)
  {
    pot_val = ANA_MAX_3V3_READ;
  }
  else if(pot_val < BRIGHTNESS_MIN_POT_READ)
  {
    // Ensures that leds can't be turned completely off
    pot_val = BRIGHTNESS_MIN_POT_READ;
  }
  duty = ((float)pot_val / ANA_MAX_3V3_READ);

  return duty;
}

void update_led_freq(uint32_t ticks)
{
  // Updating OCRA which controls the period
  led_tick_period_s = (ticks > 255) ? 0xFF : ticks;
  OCR0A = led_tick_period_s;
}

void update_mag_freq(uint32_t ticks)
{
  // Updating OCRA which controls the period
  mag_tick_period_s = (ticks > 255) ? 0xFF : ticks;
  OCR2A = mag_tick_period_s;
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
  const uint32_t prescaler = SYS_PRESCALER;

  #if defined(DEBUG)
    // initialize serial communication at 2400 bits per second:
    Serial.begin(BAUD_RATE * prescaler);
  #endif // DEBUG
}

void init_state(void)
{
  // Set the System Prescaler
  noInterrupts();
  CLKPR = _BV(CLKPCE);  // enable change of the clock prescaler
  CLKPR = 4;  // divide frequency by 16(=4)
  interrupts();

  bStarted_g = false;
}

void blink_board_led(uint32_t blinks)
{
  #if defined(DEBUG)
    if (blinks == 1)
    {
      digitalWrite(BRD_LED_PIN, HIGH);
      my_delay(BLINK_PERIOD);
      digitalWrite(BRD_LED_PIN, LOW);
    }
    else
    {
      for (int i=0; i<blinks; i++)
      {
        digitalWrite(BRD_LED_PIN, HIGH);
        my_delay(BLINK_PERIOD);
        digitalWrite(BRD_LED_PIN, LOW);
        my_delay(BLINK_PERIOD);
      }
    }
  #endif // DEBUG
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

  // Turn off PWMs (LED and MAGNET)
  analogWrite(PWM_LED_PIN, 0);
  analogWrite(PWM_MAG_PIN, 0);

  #if defined(DEBUG)
    // Initialize Board LED for debug
    pinMode(BRD_LED_PIN, OUTPUT);
    digitalWrite(BRD_LED_PIN, LOW);

    // As a visual que to know that we have started, toggle the pin 3 times
    blink_board_led(3);
  #endif //DEBUG
}

void init_timers(void)
{
  events_g = NO_PENDING_EVENTS;
  
  ITimer1.init();

  // Initialize Timer0 and Timer1
  // Updating WGM 3 -> 7 (=FastPwm, Top=OCR#A, Update @ bottom)
  TCCR0A |= 3;
  TCCR0B |= 0x8;
  TCCR2A |= 3;
  TCCR2B |= 0x8;

  // Up the prescaler to make it much slower
  //   CLKsys (16Mhz) prescaled(16) = 500kHz. CLKio (1MHz) prescaled(64) = 15625 Hz --> Range of (61Hz - 15625Hz) -> 195 ticks = 80.1Hz, 196 ticks = 79.7Hz (~.4Hz / tick)
  // Timer 0 Scaler Options: 1(DIV_1), 2(DIV_8), 3(DIV_64), 4(DIV_256), 5(DIV_1024)
  TCCR0B &= 0xF8; // clear the CS field
  TCCR0B |= 3; // 3(DIV_64)
  // Timer 2 Scaler Options: 1(DIV_1), 2(DIV_8), 3(DIV_32), 4(DIV_64), 5(DIV_128), 6(DIV_256), 7(DIV_1024)
  TCCR2B &= 0xF8; // clear the CS field
  TCCR2B |= 4; // 4(DIV_64)

  // Initializing Periods of Timer compares (PWMs) to the slowest frequency
  OCR0A = 0xFF;
  OCR2A = 0xFF;

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
  // print out system prescaler
  my_printf("System");
  my_printf("----------------------------------- ");
  my_printf("CLKPR = 0x" + String(CLKPR, HEX));
  my_printf("----------------------------------- ");
  
  print_timer_0_cfg();
  print_timer_1_cfg();
  print_timer_2_cfg();
}