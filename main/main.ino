#include "ex_cpp.hh"

void setup() {
  // Initialize Variables

  // Initialize Sense Pin

  // Initialize Backup Start Pin

  // Initialize Timers
  init_timers();

  // Initialize PWMs
}

void loop() {
  // If an event is present, perform the necessary action
  if (events_g != NO_PENDING_EVENTS)
  {
    process_event();
  }
}
