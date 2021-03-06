#ifndef LINK_HOOK_MODULE_MESSAGE_H
#define LINK_HOOK_MODULE_MESSAGE_H

#define __builtin_va_start(v,l)
#define __builtin_va_end(v)

#define LHM_DEBUG_ON 1
#define DEBUG_SERIAL_CLASS SERIAL_CLASS
#define COM_SERIAL_CLASS SERIAL2_CLASS

#include <Arduino.h>
#include <USBSerial.h>
#include "LinkHookModuleDataTable.h"
// #include <stdio.h>
// #include <stdarg.h>

enum class CommandType {
    ERROR = 0,

    HINGE_POWER_OFF = 1,
    HINGE_TAKE_OFF = 2,
    HINGE_LANDING = 3,
    HINGE_SWING_REDUCTION = 4,
    
    HOOK_POWER_OFF = 6,
    HOOK_CLOSE = 7,
    HOOK_OPEN = 8
};

enum class ExecutionResult {
    Failed = 0,
    Successful = 1
};

class LHMMessage
{
    public:
    LHMMessage(DEBUG_SERIAL_CLASS &debugSerial, COM_SERIAL_CLASS &comSerial);
    int readCommandIn();
    void sendCommandFeedback(CommandType cmd, bool isSuccessful);
    void sendCommandFeedback(int cmd, bool isSuccessful);
    void sendDebugMessage(char *fmt, ...);

    private:
    DEBUG_SERIAL_CLASS &debugSerial;
    COM_SERIAL_CLASS &comSerial;
    String getSerialMessage();
    int parseSerialMessage(String message);
    static bool cmdEqualTo(int currentCmd, CommandType compareToCmd);
};

#endif