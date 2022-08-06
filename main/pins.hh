//
// Filename: pins.hh
// Author: Beau Roland
// Details: Pinout Header file
// Date: 7/31/22
//

#ifndef PINS_H
#define PINS_H

//---------
// DEFINES, CONSTANTS, LITERALS

#ifndef LED_BUILTIN
  #define LED_BUILTIN	5
#endif
#define BRD_LED_PIN LED_BUILTIN

                            // Timer 1 should be used for event timing
#define PWM_LED_PIN     6   // Timer 0 Compare A
#define PWM_MAG_PIN     11  // Timer 2 Compare A

#define MAG_DETECT_PIN  2
#define BK_UP_PIN       3

#endif // PINS_H
