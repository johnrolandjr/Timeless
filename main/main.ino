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

void start_animation(void)
{
  bStarted = true;
  showtime_count_g = SHOWTIME_ITER;

  // Setup up and start the PWMs (LED and Magnet)
  start_pwm();
}

void stop_animation(void)
{
  bStarted = false;
  backup_sw_cnt_g = 0;

  // Turn off PWMs (LED and MAGNET)
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
  if (bStarted)
  {
    if (showtime_expired())
    {
      // Showtime over, stop the animation
      stop_animation();
    }
    else
    {
      // If it's still showtime, update the led brightness and pwm accordingly
      update_led_pwm();
    }
    // Wait for next time to see if showtime is done
    delay_ms(UPDATE_MS);
  }
  else
  {
    // Currently not animating. Check to see whether it's time to start now.
    if (magnet_detected() || (duration_sw_held() > BACKUP_SW_HELD_ITER))
    {
      start_animation();
    }
    // Wait for next check
    delay_ms(SHOWTIME_CHECK_MS);
  }
}
