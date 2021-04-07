#ifndef LONG_SHORT_PRESS_BUTTON_H
#define LONG_SHORT_PRESS_BUTTON_H

#include <Arduino.h>

const int SHORT_PRESS_TIME = 500;
const int LONG_PRESS_TIME = 500;

enum class LSButtonState
{
    RELEASED = 0,
    SHORT_PRESSED,
    LONG_PRESSED,
    PRESSED
};

class LongShortPressButton
{
public:
    inline LongShortPressButton(uint8_t buttonPin) : pin(buttonPin) {}
    void setup();
    LSButtonState getState();

private:
    uint8_t pin;
    int lastState = LOW;
    int currentState;
    unsigned long pressedTime = 0;
    unsigned long releasedTime = 0;
};
#endif