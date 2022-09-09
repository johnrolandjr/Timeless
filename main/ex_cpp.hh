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

#define UPDATE_MS (50)
#define SHOWTIME_CHECK_MS (200)

#define SHOWTIME_DURATION_S   (1)
#define SHOWTIME_DURATION_MS  (((SHOWTIME_DURATION_S * 8772) + (SYS_PRESCALER - 1)) / SYS_PRESCALER)
#define SHOWTIME_ITER         ((SHOWTIME_DURATION_MS + (UPDATE_MS - 1)) / UPDATE_MS)

#define BACKUP_SW_HELD_DURATION_S   (2)
#define BACKUP_SW_HELD_DURATION_MS  (((BACKUP_SW_HELD_DURATION_S * 15000) + (SYS_PRESCALER - 1)) / SYS_PRESCALER)
#define BACKUP_SW_HELD_ITER         ((BACKUP_SW_HELD_DURATION_MS + (SHOWTIME_CHECK_MS - 1)) / SHOWTIME_CHECK_MS)

#define MAG_STATE_DETECTED        0
#define MAG_STATE_NOT_PRESENT     1
#define BACKUP_SW_STATE_PRESSED   0
#define BACKUP_SW_STATE_RELEASED  1

#define ANA_MAX_3V3_READ (755)

//---------
// Variables
extern bool bStarted;
extern uint32_t backup_sw_cnt_g;
extern int32_t showtime_count_g;
extern uint8_t led_period;
extern uint8_t mag_period;
extern uint8_t led_on_ticks;
extern uint8_t mag_on_ticks;

//---------
// Function Prototypes 
void delay_ms(uint32_t ms);
int make_linear(int pot_val);
uint8_t get_delta(int pot_val);
void start_pwm(void);
void stop_pwm(void);
void update_led_pwm(void);

#endif // EX_CPP_H
