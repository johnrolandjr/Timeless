#include "ex_cpp.hh"

void setup() {

  // Initialize Variables and system clock
  init_state();

  #if defined(DEBUG)
    // Init Serial Debug
    init_serial();
  #endif

  my_printf("Initializing");

  // Initialize Timers
  init_timers();

  // Initialize GPIO Pins
  init_pins();
}

void loop() {
  // If an event is present, perform the necessary action
  if (events_g != NO_PENDING_EVENTS)
  {
    process_event();
  }
}
