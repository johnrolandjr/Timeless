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

// Timer Compare Outputs
#define PWM_LED_PIN     6
#define PWM_MAG_PIN     11
#define MAG_DETECT_PIN  2
#define BK_UP_PIN       3

#endif // PINS_H
