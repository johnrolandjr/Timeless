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
// Timers enabled in this project
#define USE_TIMER_1 true
#define USE_TIMER_2 false
#define USE_TIMER_3 false
#define USE_TIMER_4 false
#define USE_TIMER_5 false

// Event values
#define TIMER_1_ISR_EVT   (1<<0)

#define TIMER_1_INTERVAL_MS 1000L
#define NO_PENDING_EVENTS 0
#define NOT_TRIGGERED 0

#define BACKUP_START_SW_HELD_ITERATION 2

//---------
// Variables
extern uint32_t events_g;
extern bool bStarted_g;

//---------
// Function Prototypes
void init_state(void);
void init_timers(void);
void process_event(void);
void process_main_loop(void);
void start_animation(void);
void stop_animation(void);
void debounce_input_pins(void);
bool magnet_detected(void);
bool switch_pressed(void);
uint32_t duration_sw_held(void);
bool showtime_expired(void);

//---------
// ISR Function Prototypes
void timer_1_handler(void);

#endif // EX_CPP_H
