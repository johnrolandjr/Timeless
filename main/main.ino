#include "ex_cpp.hh"
#include "pins.hh"

void setup() {
  // Setup System Prescaler
  CLKPR = _BV(CLKPCE);  // enable change of the clock prescaler
  CLKPR = 4;  // divide frequency by 16(=4)
  
  // WGM set to FastPWM
  TCCR0A = 0x3;
  TCCR0B = 0x08;
  TCCR2A = 0x3;
  TCCR2B = 0x08;

  // WGM set to normal for Timer 1 as that will be our slow system clock
  TCCR1A = 0;
  TCCR1B = 0;

  // Initialize led pwm pin as digital output
  pinMode(PWM_LED_PIN, OUTPUT);

  // Initialize mag pwm pin as digital output
  pinMode(PWM_MAG_PIN, OUTPUT);

  // Set the pwm timers clocks to be disabled as a start
  stop_pwm();

  // For debugging purposes, start the timer
  start_pwm();
}

void loop() {
  delay_ms(100);
  update_led_pwm();
}
