#include "ex_cpp.hh"
#include "pins.hh"

// ---------------------------------------------------------------------------------------------------------------
// Application prototypes
bool magnet_detected(void);
uint32_t duration_sw_held(void);
bool showtime_expired(void);
void start_animation(void);
void stop_animation(void);

// ---------------------------------------------------------------------------------------------------------------
// Application functions
bool magnet_detected(void)
{
  int state = digitalRead(MAG_DETECT_PIN);
  
  return (state == MAG_STATE_DETECTED);
}

void start_animation(void)
{
  bStarted = true;
  start_pwm();
}

void stop_animation(void)
{
  bStarted = false;
  stop_pwm();
}

// ---------------------------------------------------------------------------------------------------------------
// Arduino Setup
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

  // Initialize magnet pin as digital input
  pinMode(MAG_DETECT_PIN, INPUT);

  // Initialize backup switch as digital input
  pinMode(BK_UP_PIN, INPUT);

  // Set the pwm timers clocks to be disabled as a start
  stop_pwm();

  // Init variables
  bStarted = false;
  backup_sw_cnt_g = 0;
}

// Arduino Main Loop
void loop() {
  uint32_t timer1_cnt_g;
  uint32_t bkup_hold_sec_g;
  uint32_t showtime_sec_g;
  
  // Pre showtime
  timer1_cnt_g=0;
  bkup_hold_sec_g=0;
  while (bStarted == false)
  {
    // Currently not animating. Check to see whether it's time to start now.
    if (magnet_detected() || (bkup_hold_sec_g >= BKUP_HOLD_DURATION))
    {
      start_animation();
    }
    // Wait for next check
    my_delay_ms(SHOWTIME_CHECK_MS);
    timer1_cnt_g++;
    if (timer1_cnt_g > BKUP_HOLD_1_SEC_CNT)
    {
      timer1_cnt_g=0;
      bkup_hold_sec_g++;
    }
  }
  
  // Showtime
  timer1_cnt_g=0;
  showtime_sec_g=0;
  while (bStarted == true)
  {
    if (showtime_sec_g >= SHOWTIME_DURATION)
    {
      // Showtime over, stop the animation
      stop_animation();
    }
    else
    {
      // Update the brightness and "slowness" if we are still in showtime
      update_led();
    }
    // Wait for next time to see if showtime is done
    my_delay_ms(UPDATE_MS);
    timer1_cnt_g++;
    if (timer1_cnt_g > SHOWTIME_1_SEC_CNT)
    {
      timer1_cnt_g=0;
      showtime_sec_g++;
    }
  }
}
