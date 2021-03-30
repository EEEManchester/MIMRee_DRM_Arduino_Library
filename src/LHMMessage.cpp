#include "LHMMessage.h"

int8_t LHMMessage::readCommandIn()
{
    uint32_t timestamp = 0;
    uint8_t index = (int)MessageType::CMD_IN;
    float value = (float)CommandType::UNKNOWN;

    mavlink.readDebug(timestamp, index, value);
    if (value < 1)
        return (int8_t)CommandType::ERROR;

    
    return (int8_t) value;
}

void LHMMessage::sendCommandFeedback(CommandType cmd, bool isSuccessful)
{
    sendCommandFeedback((uint8_t)cmd, isSuccessful);
}

void LHMMessage::sendCommandFeedback(uint8_t cmd, bool isSuccessful)
{
    uint32_t timestamp = 0;
    mavlink.sendDebug((int)MessageType::FB_CMD_SENT, cmd, (float)isSuccessful);
    DEBUG_SERIAL.printf("LHMMessage::sendCommandFeedback:: %i:%i\n", cmd, isSuccessful);
}

void LHMMessage::sendCommandFeedbackReception(uint8_t cmd, bool isSuccessful)
{
    uint32_t timestamp = 0;
    mavlink.sendDebug(timestamp, (int)MessageType::FB_CMD_RECEPTION, (float)isSuccessful);
    DEBUG_SERIAL.printf("LHMMessage::sendCommandFeedbackReception:: %i:%i\n", cmd, isSuccessful);
}

void LHMMessage::sendStatusMessage(HookStatus hookStatus, HingeStatus hingeStatus, uint8_t payload)
{
    char name[10] = "LHMS";
    uint32_t timestamp = 0;

    mavlink.sendDebugVect(*name, timestamp, (float)hookStatus, (float)hingeStatus, (float)payload);
    DEBUG_SERIAL.println("LHMMessage::sendStatusMessage:: Message sent.");
}