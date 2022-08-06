#include "ex_cpp.hh"

void setup() {
  #if defined(DEBUG)
    // Init Serial Debug
    init_serial();
    my_printf("Initializing");
  #endif
  
  // Initialize Variables
  init_state();

  // Initialize Timers
  init_timers();

  // Initialize GPIO Pins
  init_pins();

  #if defined(DEBUG)
    // Print Timer state
    print_timer_cfg();
  #endif // DEBUG
}

void loop() {
  // If an event is present, perform the necessary action
  if (events_g != NO_PENDING_EVENTS)
  {
    process_event();
  }
}
