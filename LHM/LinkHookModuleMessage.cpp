#include "LinkHookModuleMessage.h"


LHMMessage::LHMMessage(USBSerial &serial) : debugSerial(serial)
{
}

HingeCommand LHMMessage::getCurrentHingeCommand()
{
    float cmd = analogRead(PIN_CMD_IN) * 1.0 / PWM_MAX_PIN_CMD;
    if (LHMMessage::cmdEqualTo(cmd, HingeCommand::POWER_OFF))
    {
        return HingeCommand::POWER_OFF;
    }
    if (LHMMessage::cmdEqualTo(cmd, HingeCommand::TAKE_OFF))
    {
        return HingeCommand::TAKE_OFF;
    }
    if (LHMMessage::cmdEqualTo(cmd, HingeCommand::LANDING))
    {
        return HingeCommand::LANDING;
    }
    if (LHMMessage::cmdEqualTo(cmd, HingeCommand::SWING_REDUCTION))
    {
        return HingeCommand::SWING_REDUCTION;
    }

    int btn1 = digitalRead(PIN_BUTTON_1);
    int btn2 = digitalRead(PIN_BUTTON_2);
    if (btn1 == HIGH && btn2 == LOW)
    {
        return HingeCommand::TAKE_OFF;
    }
    if (btn1 == LOW && btn2 == HIGH)
    {
        return HingeCommand::LANDING;
    }
    if (btn1 == HIGH && btn2 == HIGH)
    {
        return HingeCommand::SWING_REDUCTION;
    }
    if (btn1 == LOW && btn2 == LOW)
    {
        return HingeCommand::POWER_OFF;
    }
}

HookCommand LHMMessage::getCurrentHookCommand()
{
    float cmd = analogRead(PIN_CMD_IN) * 1.0 / PWM_MAX_PIN_CMD;
    if (LHMMessage::cmdEqualTo(cmd, HookCommand::POWER_OFF))
    {
        return HookCommand::POWER_OFF;
    }
    if (LHMMessage::cmdEqualTo(cmd, HookCommand::CLOSE))
    {
        return HookCommand::CLOSE;
    }
    if (LHMMessage::cmdEqualTo(cmd, HookCommand::OPEN))
    {
        return HookCommand::OPEN;
    }
}

void LHMMessage::sendDebugMessage(char *fmt, ...)
{
    #if LHM_DEBUG_ON
    va_list va;
    va_start(va, fmt);
    LHMMessage::debugSerial.printf(fmt, va)
    va_end(va);
    #endif
}

bool LHMMessage::cmdEqualTo(float cmd, HingeCommand hCmd)
{
    return abs(cmd - (int)hCmd / 100.0) < 0.05;
}

int LHMMessage::sendCommandFeedback(HingeCommand cmd, bool isSuccessful, char message[])
{
    int pwm = sendCommandFeedback((int)cmd, isSuccessful, message);
    if (isSuccessful)
    {
        setStatusLEDs(cmd);
    }
    return pwm;
}

int LHMMessage::sendCommandFeedback(HookCommand cmd, bool isSuccessful, char message[])
{
    int pwm = sendCommandFeedback((int)cmd, isSuccessful, message);
    if (isSuccessful)
    {
        setStatusLEDs(cmd);
    }
    return pwm;
}

int LHMMessage::sendCommandFeedback(int cmd, bool isSuccessful, char message[])
{
    int pwm_name = cmd / 100.0 * PWM_MAX_PIN_FBK;
    int pwm_result = isSuccessful ? 0.66 * PWM_MAX_PIN_FBK : 0.33 * PWM_MAX_PIN_FBK;
    analogWrite(PIN_CMD_FBK_NAME, pwm_name);
    analogWrite(PIN_CMD_FBK_RESULT, pwm_result);
    if (message != NULL)
    {
        sendDebugMessage(message);
    }
    else
    {
        sendDebugMessage("LHMMessage::sendCommandFeedback::Pin[%i] = %i | Pin[%i] = %i", PIN_CMD_FBK_NAME, pwm_name, PIN_CMD_FBK_RESULT, pwm_result);
    }
    return pwm_name;
}

void LHMMessage::setStatusLEDs(HingeCommand cmd)
{
    int led1Val, led2Val = LOW;
    switch (cmd)
    {
    case HingeCommand::POWER_OFF:
        break;
    case HingeCommand::TAKE_OFF:
        led1Val = HIGH;
        break;
    case HingeCommand::LANDING:
        led2Val = HIGH;
        break;
    case HingeCommand::SWING_REDUCTION:
        led1Val = HIGH;
        led2Val = HIGH;
        break;
    }
    digitalWrite(PIN_LED_1, led1Val);
    digitalWrite(PIN_LED_2, led2Val);
}

void LHMMessage::setStatusLEDs(HookCommand cmd)
{
    switch (cmd)
    {
    case HookCommand::POWER_OFF:
        digitalWrite(PIN_LED_0, LOW);
        break;
    case HookCommand::CLOSE:
    case HookCommand::OPEN:
        digitalWrite(PIN_LED_0, HIGH);
        break;
    }
}