#include "LongShortPressButton.h"

void LongShortPressButton::setup()
{
    pinMode(pin, INPUT_PULLDOWN);
}

LSButtonState LongShortPressButton::getState()
{
    currentState = digitalRead(pin);
    LSButtonState state = LSButtonState::PRESSED;

    if (lastState == LOW && currentState == HIGH)
    { // button is just pressed
        pressedTime = millis();
        state = LSButtonState::PRESSED;
    }
    else if (lastState == HIGH && currentState == LOW)
    { // button is just released
        releasedTime = millis();

        long pressDuration = releasedTime - pressedTime;

        if (pressDuration < SHORT_PRESS_TIME)
            state = LSButtonState::SHORT_PRESSED;
        else if (pressDuration > LONG_PRESS_TIME)
            state = LSButtonState::LONG_PRESSED;
    }
    else if (lastState == HIGH && currentState == HIGH)
    {
        state = LSButtonState::PRESSED;
    }
    else
    {
        state = LSButtonState::RELEASED;
    }

    // save the the last state
    lastState = currentState;
    return state;
}