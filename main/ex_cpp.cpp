#include <Arduino.h>
#include "ex_cpp.hh"
#include "pins.hh"

bool bStarted = false;
uint32_t backup_sw_cnt_g;
uint8_t led_period;
uint8_t mag_period;
uint8_t led_on_ticks;
uint8_t mag_on_ticks;
int32_t showtime_count_g;

void delay_ms(uint32_t ms)
{
  // Timer 1 is stopped by default
  // Reset timer count = 0
  TCNT1 = 0;

  // Start timer 1
  TCCR1B = 5;

  // Wait until low byte is greater than requested number of ms
  while(TCNT1L < ms){};

  // Stop timer 1
  TCCR1B = 0;
}

int make_linear(int pot_val)
{
  int out;
  int frac = ANA_MAX_3V3_READ / 4;
  if (pot_val < 0)
  {
    out = 0;
  }
  else if (pot_val < 50)
  {
    out = map(pot_val, 0, 50, 0, frac);
  }
  else if (pot_val < 117)
  {
    out = map(pot_val, 50, 117, frac, 2*frac);
  }
  else if (pot_val < 340)
  {
    out = map(pot_val, 117, 340, 2*frac, 3*frac);
  }
  else
  {
    out = map(pot_val, 340, ANA_MAX_3V3_READ, 3*frac, 4*frac);
  }
  return out;
}

uint8_t get_delta(int pot_val)
{
  uint32_t delta = 0;
  uint8_t ret = 0;

  // Before we do anything, make the pot value more linear
  pot_val = make_linear(pot_val);
  
  if (pot_val < 0)
  {
    pot_val = 0;
  }

  if (pot_val >= ANA_MAX_3V3_READ)
  {
    ret = 0xFF;
  }
  else
  {
    delta = (uint32_t) pot_val;
    delta = ((delta << 8) / ANA_MAX_3V3_READ);
    ret = (uint8_t) delta;
  }
  
  // get_delta should never return 0 or 255 as we always want some sort of PWM
  if (ret == 0)
  {
    ret = 1;
  }
  else if (ret >= 0xFF)
  {
    ret = 0xFE;
  }
  return ret;
}

void start_pwm(void)
{
  uint8_t period_ticks, period_shift, period_delta;
  // In the future, we may read an analog voltage and change the delta periods between magnet and led.
  // CLKsys (16Mhz) prescaled(16) = 1MHz. CLKio (1MHz) prescaled(64) = 15625 Hz --> Range of (61Hz - 15625Hz)
  period_ticks = 195; // 195 measured 79.5Hz
  period_shift = 0; // (~0.3Hz / tick)

  int delta_pot_val = analogRead(DELTA_POT_PIN);
  uint8_t period_shift_duty;
  period_shift_duty = get_delta(delta_pot_val); // (~0.3Hz / tick)
  int brightness_pot_val = analogRead(BRIGHTNESS_POT_PIN);
  uint8_t led_duty = get_delta(brightness_pot_val);

  // Update the frequencies prior to turning them on
  period_delta = map(period_shift_duty, 0, 0xFF, 0, 60);
  led_period = period_ticks + period_shift + period_delta;
  led_on_ticks = map(led_duty, 0, 255, 0, led_period);
  mag_period = period_ticks;

  // Set LED PWM according to this moment's voltage reading on the potentiometer
  OCR0A = led_period; // period
  OCR0B = led_on_ticks; // duty

  // Set Magnet PWM duty cycle to 50%
  OCR2A = mag_period-1; // period
  OCR2B = (mag_period >> 1); // duty

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

void update_led_pwm(void)
{
  uint8_t period_ticks, period_shift, period_delta;
  // In the future, we may read an analog voltage and change the delta periods between magnet and led.
  // CLKsys (16Mhz) prescaled(16) = 1MHz. CLKio (1MHz) prescaled(64) = 15625 Hz --> Range of (61Hz - 15625Hz)
  period_ticks = 195; // 195 measured 79.5Hz
  period_shift = 0; // (~0.3Hz / tick)

  int delta_pot_val = analogRead(DELTA_POT_PIN);
  uint8_t period_shift_duty;
  period_shift_duty = get_delta(delta_pot_val); // (~0.3Hz / tick)
  int brightness_pot_val = analogRead(BRIGHTNESS_POT_PIN);
  uint8_t led_duty = get_delta(brightness_pot_val);

  // Update the frequencies prior to turning them on
  period_delta = map(period_shift_duty, 0, 0xFF, 0, 60);
  led_period = period_ticks + period_shift + period_delta;
  led_on_ticks = map(led_duty, 0, 255, 0, led_period);

  // To avoid a flickering, wait until timer is 0
  while(TCNT0 != 0){};
  
  // Set LED PWM according to this moment's voltage reading on the potentiometer
  OCR0A = led_period; // period
  OCR0B = led_on_ticks; // duty
}
