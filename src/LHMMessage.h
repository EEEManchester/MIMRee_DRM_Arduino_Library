#ifndef MIMREE_DRM_CONTROLLER_LHM_MESSAGE_H
#define MIMREE_DRM_CONTROLLER_LHM_MESSAGE_H

#define __builtin_va_start(v,l)
#define __builtin_va_end(v)

#define LHM_DEBUG_ON 1

#include <Arduino.h>
#include <USBSerial.h>

#include "LHMDataTable.h"
#include "mavlink/MAVLink.h"

class LHMMessage
{
    public:
    LHMMessage(): mavlink(MAVLink(COM_SERIAL)){}
    int8_t readCommandIn();
    void sendCommandFeedback(CommandType cmd, bool isSuccessful);
    void sendCommandFeedback(uint8_t cmd, bool isSuccessful);
    void sendStatusMessage(HookStatus hookStatus, HingeStatus hingeStatus, uint8_t payload);

    private:
    MAVLink &mavlink;
    String getSerialMessage();
    int8_t parseSerialMessage(String message);
    void sendCommandFeedbackReception(uint8_t cmd, bool isSuccessful);
};

#endif