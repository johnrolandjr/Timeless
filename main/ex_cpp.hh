//
// Filename: ex_cpp.hh
// Author: Beau Roland
// Details: Example C++ Header file
// Date: 7/31/22
//

#ifndef EX_CPP_H
#define EX_CPP_H

//---------
//DEFINES, CONSTANTS, LITERALS
#define SYS_PRESCALER (16)

#define UPDATE_MS (100)
#define SHOWTIME_CHECK_MS (100)

#define SHOWTIME_DURATION_S   (4)
#define SHOWTIME_DURATION     ((uint32_t)((float)SHOWTIME_DURATION_S / 1.128))
#define SHOWTIME_1_SEC_CNT    ((1000 + (UPDATE_MS - 1)) / UPDATE_MS)

#define BKUP_HOLD_DURATION_S  (2)
#define BKUP_HOLD_DURATION    ((uint32_t)((float)BKUP_HOLD_DURATION_S / 1.128))
#define BKUP_HOLD_1_SEC_CNT   ((1000 + (SHOWTIME_CHECK_MS - 1)) / SHOWTIME_CHECK_MS)

#define MAG_STATE_DETECTED        0
#define MAG_STATE_NOT_PRESENT     1
#define BACKUP_SW_STATE_PRESSED   0
#define BACKUP_SW_STATE_RELEASED  1

#define ANA_MAX_3V3_READ (755)

//---------
// Variables
extern bool bStarted;
extern uint32_t backup_sw_cnt_g;
extern int showtime_count_g;
extern uint8_t led_period;
extern uint8_t mag_period;
extern uint8_t led_on_ticks;
extern uint8_t mag_on_ticks;

//---------
// Function Prototypes 
void my_delay_ms(uint32_t ms);
int make_linear(int pot_val);
uint8_t get_delta(int pot_val);
void start_pwm(void);
void stop_pwm(void);
void update_led_pwm(void);

#endif // EX_CPP_H
