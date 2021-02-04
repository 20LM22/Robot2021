#ifndef ShooterLEDs_h
#define ShooterLEDs_h

#include "Arduino.h"
#include <Adafruit_NeoPixel.h>

class ShooterLEDs {
  public:
    static void initialize(byte pin, byte numLEDs, byte brightness);
    static void refresh(byte mode, byte value);

  private:
    // constant RGB values for different colors
    static const uint32_t off;
    static const uint32_t red;
    static const uint32_t orange;
    static const uint32_t yellow;
    static const uint32_t green;
    static const uint32_t blue;
    static const uint32_t purple;
    static const uint32_t white;
    // LED strip object
    static Adafruit_NeoPixel strip;

    // for timing animations
    static unsigned long timestamp;
    // global counter variable
    static byte counter;
    // stores previous mode to check if the new one is different
    static byte prevMode;

    // turns all LEDs off
    static void allOff();
    // shows the percent speed of the flywheel based on its setpoint
    static void flywheelPercent(byte percent);
};

#endif
