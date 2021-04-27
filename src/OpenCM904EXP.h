#ifndef MIMREE_DRM_CONTROLLER_OPENCM904EXP_HH
#define MIMREE_DRM_CONTROLLER_OPENCM904EXP_HH

#include <Arduino.h>
#include "utilities/LEDController.h"

const uint8_t PIN_DXL_DIR = 22;
const uint8_t PIN_LED_STAT = 14;
const uint8_t PIN_LED_RED = 18;
const uint8_t PIN_LED_GREEN = 19;
const uint8_t PIN_LED_BLUE = 20;
const uint8_t PIN_BUTTON_1 = 16;
const uint8_t PIN_BUTTON_2 = 17;

class OpenCM904EXP
{
public:
    inline OpenCM904EXP() : ledStat(LEDController(PIN_LED_STAT, true)),
                     ledRed(LEDController(PIN_LED_RED, true)),
                     ledGreen(LEDController(PIN_LED_GREEN, true)),
                     ledBlue(LEDController(PIN_LED_BLUE, true))
    {
    }
    virtual ~OpenCM904EXP() {}
    LEDController ledStat;
    LEDController ledRed;
    LEDController ledGreen;
    LEDController ledBlue;

    inline void setupOnBoardDevices()
    {
        ledStat.setup();
        ledRed.setup();
        ledGreen.setup();
        ledBlue.setup();
        pinMode(PIN_BUTTON_1, INPUT_PULLDOWN);
        pinMode(PIN_BUTTON_2, INPUT_PULLDOWN);

        ledStat.set(LED_OFF_REVERSED);
        ledRed.set(LED_OFF_REVERSED);
        ledGreen.set(LED_OFF_REVERSED);
        ledBlue.set(LED_OFF_REVERSED);
        
    }

    inline void loopAllLED()
    {
        ledStat.loop();
        ledRed.loop();
        ledGreen.loop();
        ledBlue.loop();
    }

    inline void togglePin(uint8_t pin) { digitalWrite(pin, !digitalRead(pin)); }
};
#endif