#ifndef LHM_MESSAGE_H
#define LHM_MESSAGE_H

#define __builtin_va_start(v,l)
#define __builtin_va_end(v)

#define LHM_DEBUG_ON 1

#include <Arduino.h>
#include <USBSerial.h>
#include "LHMDataTable.h"

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
    int8_t readCommandIn();
    void sendCommandFeedback(CommandType cmd, bool isSuccessful);
    void sendCommandFeedback(uint8_t cmd, bool isSuccessful);

    private:
    String getSerialMessage();
    int8_t parseSerialMessage(String message);
};

#endif