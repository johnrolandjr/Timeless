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
uint32_t showtime_count_g;

uint8_t led_period;
uint8_t mag_period;
uint8_t led_on_ticks;
uint8_t mag_on_ticks;

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
    else
    {
      // Even if the it's not time to stop the show, update the the frequencies and pwms (brightness)
      update_show();
    }
  }
  else
  {
    // Currently not animating. Check to see whether it's time to start now.
    #if defined(DISABLE_BACKUP_SWITCH)
      if (magnet_detected())
    #else
      //if (magnet_detected() || (duration_sw_held() > BACKUP_SW_HELD_DURATION))
      if (true)
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

  // Enable the timers to begin PWM Output
  start_pwm();

  #if defined(DEBUG)
    // As a visual que, blink led 1 time
    blink_board_led(1);
  #endif // DEBUG
}

void update_show(void)
{
  int8_t period_ticks, period_shift, period_delta;
  // In the future, we may read an analog voltage and change the delta periods between magnet and led.
  // CLKsys (16Mhz) prescaled(16) = 1MHz. CLKio (1MHz) prescaled(64) = 15625 Hz --> Range of (61Hz - 15625Hz)
  period_ticks = 195; // 195 measured 79.5Hz
  period_shift = 0; // (~0.3Hz / tick)

  // Read Potentiometers
  int delta_pot_val = analogRead(DELTA_POT_PIN);
  period_delta = get_delta(delta_pot_val); // (~0.3Hz / tick)
  int brightness_pot_val = analogRead(BRIGHTNESS_POT_PIN);
  float duty = get_brightness(brightness_pot_val);

  // Update the frequencies prior to turning them on
  led_period = period_ticks + period_shift + period_delta;
  //mag_period = period_ticks + period_shift;
  //OCR0A = led_period;
  //OCR2A = mag_period;

  // Set LED PWM according to this moment's voltage reading on the potentiometer
  led_on_ticks = (uint8_t)(led_period * duty);
  //OCR0B = led_on_ticks;

  // Set duty cycle of Magnet PWM to 50%
  // B is tied to the output
  //OCR2B = (mag_period >> 1);

  // Only updating the LED PWM signal
  // Also, updating it at t=0 so that it is glitch free
  while(TCNT0 != 0){}
  OCR0A = led_period;
  OCR0B = led_on_ticks;
}

void stop_animation(void)
{
  my_printf("Stop Animation");
  bStarted_g = false;
  backup_sw_cnt_g = 0;

  // Enable the timers to begin PWM Output
  stop_pwm();
  
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
  percent = ((float)pot_val / (ANA_MAX_3V3_READ + 10)); // Adding 10 so that we can't get 100% pwm

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
  duty = ((float)pot_val / (ANA_MAX_3V3_READ + 10)); // Adding 10 so that we can't get 100% pwm

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
  if (--showtime_count_g == 0)
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
  led_period = 0xFF; // Initializing to slowest frequency
  mag_period = 0xFF;
  led_on_ticks = 0x80; // Initializing to 50% duty cycle
  mag_on_ticks = 0x80;
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

  // Initializing LED but OC0n disconnected
  TCCR0A |= _BV(WGM01) | _BV(WGM00);
  TCCR0B |= _BV(WGM02);
  
  // Initializing PWM but OC2n disconnected
  TCCR2A |= _BV(WGM21) | _BV(WGM20);
  TCCR2B |= _BV(WGM22);

  // Initialize led pwm pin as digital output
  pinMode(PWM_LED_PIN, OUTPUT);

  // Initialize mag pwm pin as digital output
  pinMode(PWM_MAG_PIN, OUTPUT);

  // Extra write to ensure Timer2 is using the correct clock
  ASSR = 0;

  // Set the pwm timers clocks to be disabled as a start
  stop_pwm();

  // Initializing Periods of Timer compares (PWMs) to the slowest frequency
  OCR0A = 0xFF;
  OCR2A = 0xFF;

  // Ensuring that Timers 0, 1, and 2 are enabled all the time
  PRR &= 0x87; // Clearing bits 6, 5, (4 reserved), and 3

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

void stop_pwm(void)
{
  // Stop the clocks
  TCCR0B &= ~(7);
  TCCR2B &= ~(7);

  // Reset Output Compare Value via Force
  // Force is only active when WGM bits are on a non PWM mode
  // Set to Waveform Generator Mode = normal mode = 0
  TCCR0A &= ~(_BV(WGM01) | _BV(WGM00)); 
  TCCR0B &= ~_BV(WGM02);
  // Reset Count
  TCNT0 = 0;
  // Set Compare Output Mode to normal mode
  TCCR0A &= ~(_BV(COM0B1) | _BV(COM0B0));
  // Force the Compare Output (since timer is not running and it's reset to 0), it should be forced low)
  TCCR0A |= _BV(FOC0B);
  TCCR0A &= ~_BV(FOC0B);
  
  // Reset Output Compare Value via Force
  // Force is only active when WGM bits are on a non PWM mode
  // Set to Waveform Generator Mode = normal mode = 0
  TCCR2A &= ~(_BV(WGM21) | _BV(WGM20)); 
  TCCR2B &= ~_BV(WGM22);
  // Reset Count
  TCNT2 = 0;
  // Set Compare Output Mode to non inverting
  TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
  // Force the Compare Output (since timer is not running and it's reset to 0), it should be forced low)
  TCCR2A |= _BV(FOC2B);
  TCCR2A &= ~_BV(FOC2B);
}

void start_pwm(void)
{
  int8_t period_ticks, period_shift, period_delta;
  // In the future, we may read an analog voltage and change the delta periods between magnet and led.
  // CLKsys (16Mhz) prescaled(16) = 1MHz. CLKio (1MHz) prescaled(64) = 15625 Hz --> Range of (61Hz - 15625Hz)
  period_ticks = 195; // 195 measured 79.5Hz
  period_shift = 0; // (~0.3Hz / tick)

  int delta_pot_val = analogRead(DELTA_POT_PIN);
  period_delta = get_delta(delta_pot_val); // (~0.3Hz / tick)

  // Update the frequencies prior to turning them on
  led_period = period_ticks + period_shift;
  mag_period = period_ticks + period_shift + period_delta;
  OCR0A = led_period-1;
  OCR2A = mag_period-1;

  // Set LED PWM according to this moment's voltage reading on the potentiometer
  int brightness_pot_val = analogRead(BRIGHTNESS_POT_PIN);
  float duty = get_brightness(brightness_pot_val);
  led_on_ticks = (uint8_t)(led_period * duty);
  OCR0B = led_on_ticks-1;

  // Set duty cycle of Magnet PWM to 50%
  // B is tied to the output
  OCR2B = (mag_period >> 1)-1;

  // Set PWM Mode to FastPWM
  // Initializing LED but OC0n disconnected
  TCCR0A |= _BV(WGM01) | _BV(WGM00);
  TCCR0B |= _BV(WGM02);
  
  // Initializing PWM but OC2n disconnected
  TCCR2A |= _BV(WGM21) | _BV(WGM20);
  TCCR2B |= _BV(WGM22);
  
  // Set Compare Output Mode to be non inverting mode
  TCCR0A |= _BV(COM0B1);
  TCCR2A |= _BV(COM2B1);

  // Start PWMs by setting the clock
  TCCR0B |= _BV(CS01) | _BV(CS00);
  TCCR2B |= _BV(CS22);
}
