#ifndef LINK_HOOK_MODULE_MESSAGE_H
#define LINK_HOOK_MODULE_MESSAGE_H

#define __builtin_va_start(v,l)
#define __builtin_va_end(v)

#define LHM_DEBUG_ON 1
#define DEBUG_SERIAL_CLASS SERIAL_CLASS
#define COM_SERIAL_CLASS SERIAL_CLASS

#include <Arduino.h>
#include <USBSerial.h>
#include "LinkHookModuleDataTable.h"
// #include <stdio.h>
// #include <stdarg.h>

enum class CommandType {
    ERROR = -1,

    RESET_DYNAMIXEL_COM = 91,
    JETTISON = 99,
    LOCK_LHM = 98,
    
    HINGE_POWER_OFF = 1,
    HINGE_TAKE_OFF = 2,
    HINGE_LANDING = 3,
    HINGE_SWING_REDUCTION = 4,
    
    HOOK_POWER_OFF = 10,
    HOOK_CLOSE = 11,
    HOOK_OPEN = 12,
};

enum class ExecutionResult {
    Failed = 0,
    Successful = 1
};

// class CommandMessage
// {
//     public:
//     CommandType &type;
//     float dataf;
//     int datai;
//     String datas;
//     CommandMessage(CommandType type, float data):
//         type(type), dataf(data){};
//     CommandMessage(CommandType type, int data):
//         type(type), datai(data){};
//     CommandMessage(CommandType type, String data):
//         type(type), datas(data){};
        

// }

class LHMMessage
{
    public:
    DEBUG_SERIAL_CLASS &debugSerial;
    LHMMessage(COM_SERIAL_CLASS &comSerial, DEBUG_SERIAL_CLASS &debugSerial);
    int readCommandIn();
    void sendCommandFeedback(CommandType cmd, bool isSuccessful);
    void sendCommandFeedback(int cmd, bool isSuccessful);

    private:
    COM_SERIAL_CLASS &comSerial;
    String getSerialMessage();
    int parseSerialMessage(String message);
    static bool cmdEqualTo(int currentCmd, CommandType compareToCmd);
};

#endif