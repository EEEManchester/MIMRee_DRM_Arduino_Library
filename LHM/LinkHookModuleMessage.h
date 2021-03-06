#ifndef LINK_HOOK_MODULE_MESSAGE_H
#define LINK_HOOK_MODULE_MESSAGE_H

#define __builtin_va_start(v,l)
#define __builtin_va_end(v)

#define LHM_DEBUG_ON 1

#include <Arduino.h>
#include <USBSerial.h>
#include "LinkHookModuleDataTable.h"
// #include <stdio.h>
// #include <stdarg.h>

enum class HingeCommand {
    POWER_OFF = 14,
    TAKE_OFF = 29,
    LANDING = 43,
    SWING_REDUCTION = 57
};

enum class HookCommand {
    POWER_OFF = 71,
    CLOSE = 86,
    OPEN = 100
};

enum class ExecutionResult {
    Failed = 33,
    Successful = 66
};

class LHMMessage
{
    public:
    LHMMessage(USBSerial &debugSerial);
    HingeCommand getCurrentHingeCommand();
    HookCommand getCurrentHookCommand();
    void sendDebugMessage(char *fmt, ...);
    int sendCommandFeedback(HingeCommand cmd, bool isSuccessful, char message[] = NULL);
    int sendCommandFeedback(HookCommand cmd, bool isSuccessful, char message[] = NULL);
    int sendCommandFeedback(int cmd, bool isSuccessful, char message[] = NULL);

    private:
    USBSerial &debugSerial;
    static bool cmdEqualTo(float currentCmd, HingeCommand compareToCmd);
    static bool cmdEqualTo(float currentCmd, HookCommand compareToCmd);
    static void setStatusLEDs(HingeCommand cmd);
    static void setStatusLEDs(HookCommand cmd);
};

#endif