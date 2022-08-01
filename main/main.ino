#include "ex_cpp.hh"

void setup() {
  Serial.println("Initializing");
  // Initialize Variables
  init_state();

  // Initialize Sense Pin

  // Initialize Backup Start Pin

  // Initialize Timers
  init_timers();

  // Initialize PWMs

  // Ensure that device is not animating
  stop_animation();
}

void loop() {
  // If an event is present, perform the necessary action
  if (events_g != NO_PENDING_EVENTS)
  {
    process_event();
  }
}
