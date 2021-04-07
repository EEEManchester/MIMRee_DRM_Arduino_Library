#ifndef MIMREE_DRM_CONTROLLER_UTL_LED_CONTROLLER_HH
#define MIMREE_DRM_CONTROLLER_UTL_LED_CONTROLLER_HH

#ifdef LED_DEBUG
#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL Serial
#endif
#define LED_DEBUG_PRINTF(fmt, ...) DEBUG_SERIAL.printf(fmt, ##__VA_ARGS__)
#define LED_DEBUG_PRINTLN(a) DEBUG_SERIAL.println(a)
#else
#define LED_DEBUG_PRINTF(fmt, ...) 
#define LED_DEBUG_PRINTLN(a)
#endif

#include <Arduino.h>

enum LEDState
{
    LED_UNKNOWN = -1,
    LED_OFF = 0,
    LED_ON = 1
};

enum LEDStateReversed
{
    LED_ON_REVERSED = 0,
    LED_OFF_REVERSED = 1
};

enum FlashTime
{
    FLASH_TIME_BLINK = 20,
    FLASH_TIME_SHORT_EXTREME = 100,
    FLASH_TIME_SHORT_NORM = 250,
    FLASH_TIME_LONG_NORM = 500,
    FLASH_TIME_LONG_EXTREME = 1000
};

class LEDFlashController
{
public:
    inline LEDFlashController() {}
    /**
     * @brief Initiate a new instance of LEDFlashController for flashing LEDs connected to OpenCM904
     * @param pin Pin ID for LED.
     * @param times Max number of cycles that the LED needs to be turned on and off. Set a negative number to flash the LED infinitely.
     * @param onTime Amount of time that the LED is turned on before being turned off again.
     * @param offTime Amount of time that the LED is turned off before being turned on again.
     */
    inline LEDFlashController(uint8_t pin, signed long times, uint32_t onTime, uint32_t offTime, bool reverseOnOff = false)
        : pin(pin), onTime(onTime), offTime(offTime), times(times * 2), ledOnDir(reverseOnOff ? 0 : 1), ledOffDir(reverseOnOff ? 1 : 0)
    {
    }

    uint8_t pin;
    uint32_t onTime = 0;
    uint32_t offTime = 0;
    signed long times = 0;
    int8_t currentState = LED_UNKNOWN;
    inline bool completed() { return times >= 0 && count >= times; }

    inline void loop()
    {
        if (completed())
        {
            return;
        }

        if (canToggle())
        {
            toggle();
        }
    }

    inline bool isIdentical(signed long _times, uint32_t _onTime, uint32_t _offTime)
    {
        return (times == _times * 2) && (onTime == _onTime) && (offTime = _offTime);
    }

    inline void reset()
    {
        LED_DEBUG_PRINTF("LEDFlashController::reset: pin %d, ON=%d | times %d/%d (%d|%d), onTime %d, offTime %d, lastToggle %d, millis %d\n", pin, ledOnDir, count, times, times>=0, count>=times, onTime, offTime, lastToggleTime, millis());
        onTime = 0;
        offTime = 0;
        times = 0;
        currentState = LED_UNKNOWN;
        lastToggleTime = 0;
        count = 0;
    }

private:
    uint32_t lastToggleTime = 0;
    signed long count = 0;
    int ledOnDir;
    int ledOffDir;

    inline bool canToggle()
    {
        if (currentState == ledOnDir)
        {
            return (millis() - lastToggleTime) > onTime;
        }
        if (currentState == ledOffDir)
        {
            return (millis() - lastToggleTime) > offTime;
        }
        if (currentState == LED_UNKNOWN)
        {
            currentState = ledOffDir;
            digitalWrite(pin, ledOffDir);
            return true;
        }
        return false;
    }

    inline void toggle()
    {
        LED_DEBUG_PRINTF("LEDFlashController::toggle: pin %d, %d->%d (ON=%d) | times %d/%d (%d|%d), onTime %d, offTime %d, lastToggle %d, millis %d\n", pin, currentState, !currentState, ledOnDir, count, times, times>=0, count>=times, onTime, offTime, lastToggleTime, millis());
        digitalWrite(pin, !currentState);
        lastToggleTime = millis();
        count++;
        switchState();
    }

    inline void switchState()
    {
        currentState = !currentState;
    }
};

class LEDController
{
public:
    inline LEDController(uint8_t pin, bool reverseOnOff = false) : pin(pin), reverseOnOff(reverseOnOff) {}

    inline void setup()
    {
        pinMode(pin, OUTPUT);
    }

    inline void set(uint8_t state)
    {
        digitalWrite(pin, state);
        ledFlashControllers.reset();
    }

    inline void toggle()
    {
        set((int)!digitalRead(pin));
    }

    /** 
     * @brief flash LEDs connected to OpenCM904. Must call `loop()` in mainloop.
     * @param times Max number of cycles that the LED needs to be turned on and off. Set a negative number to flash the LED infinitely.
     * @param onTime Amount of time that the LED is turned on before being turned off again.
     * @param offTime Amount of time that the LED is turned off before being turned on again.
     * @param forceReset If true, reset flash controller even if settings are identical to the existing ones; otherwise, identical settings will be ignored.
     */
    inline void flash(uint32_t times, uint32_t onTime, uint32_t offTime, bool forceReset = false)
    {
        if (!forceReset && !ledFlashControllers.completed() && ledFlashControllers.isIdentical(times, onTime, offTime))
        {
            LED_DEBUG_PRINTF("LEDController::flash: flash not set: pin[%d], times[%d], onTime[%d], offTime[%d]\n", pin, times, onTime, offTime);
            return;
        }
        ledFlashControllers = LEDFlashController(pin, times, onTime, offTime, reverseOnOff);
        LED_DEBUG_PRINTF("LEDController::flash: flash set: pin[%d], times[%d], onTime[%d], offTime[%d]\n", pin, times, onTime, offTime);
    }

    inline void loop()
    {
        ledFlashControllers.loop();
    }

    inline void syncBlink(uint8_t times, uint16_t onTime, uint16_t offTime)
    {
        uint8_t on = reverseOnOff ? (uint8_t)LED_ON_REVERSED : (uint8_t)LED_ON;
        uint8_t off = reverseOnOff ? (uint8_t)LED_OFF_REVERSED : (uint8_t)LED_OFF;
        for (uint8_t i = 0; i < times; i++)
        {
            digitalWrite(pin, on);
            delay(onTime);
            digitalWrite(pin, off);
            delay(offTime);
        }
    }

private:
    LEDFlashController ledFlashControllers;
    uint8_t pin;
    bool reverseOnOff;
};
#endif