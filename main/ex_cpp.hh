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

// Timers enabled in this project
#define USE_TIMER_1 true
#define USE_TIMER_2 false
#define USE_TIMER_3 false
#define USE_TIMER_4 false
#define USE_TIMER_5 false

// Event values
#define TIMER_1_ISR_EVT   (1<<0)

#define TIMER_1_INTERVAL_MS (200L / SYS_PRESCALER)
#define NO_PENDING_EVENTS 0
#define NOT_TRIGGERED 0

#define SHOWTIME_DURATION_S   (5)
#define SHOWTIME_DURATION_MS  (SHOWTIME_DURATION_S * 1000 / SYS_PRESCALER)
#define SHOWTIME_DURATION     (SHOWTIME_DURATION_MS / TIMER_1_INTERVAL_MS)

#define BACKUP_SW_HELD_DURATION_S   (1)
#define BACKUP_SW_HELD_DURATION_MS  (BACKUP_SW_HELD_DURATION_S * 1000 / SYS_PRESCALER)
#define BACKUP_SW_HELD_DURATION     (BACKUP_SW_HELD_DURATION_MS / TIMER_1_INTERVAL_MS)

#define MAG_STATE_DETECTED        0
#define MAG_STATE_NOT_PRESENT     1
#define BACKUP_SW_STATE_PRESSED   0
#define BACKUP_SW_STATE_RELEASED  1

#define DISABLE_BACKUP_SWITCH

//#define DEBUG
// DIFFERENT Print macros determined whether debug is enabled
#if defined(DEBUG)
  #define my_printf(...) Serial.println(__VA_ARGS__)
#else
  #define my_printf(...) ;
#endif // DEBUG

#define BLINK_PERIOD_MS (150)
#define BLINK_PERIOD    ((uint32_t)((float)BLINK_PERIOD_MS * 1000))
#define my_delay(...) delayMicroseconds((uint32_t)((__VA_ARGS__) / SYS_PRESCALER))

#define DUTY_CYCLE  (0.5f)

#define BAUD_RATE   (9600)

// Aproximately 0.3Hz per tick
// 20Hz -> 60ticks
#define MAX_DELTA   (60)

#define ANA_MAX_3V3_READ (795)
#define BRIGHTNESS_MIN_POT_READ (200)

//---------
// Variables
extern uint32_t events_g;
extern bool bStarted_g;

//---------
// Function Prototypes 
void init_serial(void);
void init_state(void);
void init_pins(void);
void init_timers(void);
void process_event(void);
void process_main_loop(void);
void start_animation(void);
void stop_animation(void);
void update_led_freq(uint32_t ticks);
void update_mag_freq(uint32_t ticks);
bool magnet_detected(void);
uint32_t duration_sw_held(void);
bool showtime_expired(void);
void blink_board_led(uint32_t blinks);

uint32_t get_delta(int pot_val);
float get_brightness(int pot_val);

void print_timer_0_cfg(void);
void print_timer_1_cfg(void);
void print_timer_2_cfg(void);
void print_timer_cfg(void);

//---------
// ISR Function Prototypes
void timer_1_handler(void);

#endif // EX_CPP_H
