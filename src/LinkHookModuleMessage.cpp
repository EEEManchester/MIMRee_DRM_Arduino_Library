#include "LinkHookModuleMessage.h"

LHMMessage::LHMMessage(DEBUG_SERIAL_CLASS &serial, COM_SERIAL_CLASS &comSerial) : debugSerial(serial), comSerial(comSerial)
{
}

int LHMMessage::readCommandIn()
{
    return parseSerialMessage(getSerialMessage());
}

String LHMMessage::getSerialMessage()
{
    if (comSerial.available() < 1)
    {
        return "";
    }
    String val = comSerial.readString();
    sendDebugMessage("LHMMessage::getSerialMessage::Incoming message received: %s", val);
    return val;
}

int LHMMessage::parseSerialMessage(String message)
{
    if (!message.startsWith(String(SERIAL_PREFIX)) || !message.endsWith(String(SERIAL_PREFIX)))
    {
        return (int)CommandType::ERROR;
    }
    if (message[1] == SERIAL_MESSAGE_TYPE_INDICATOR_CMD)
    {
        String command = message.substring(2, message.length() - 2);
        int val = command.toInt();
        sendDebugMessage("LHMMessage::parseSerialMessage::(Extracted) %s -> (Converted) %i", command, val);
    }
    return (int)CommandType::ERROR;
}

void LHMMessage::sendDebugMessage(char *fmt, ...)
{
#if LHM_DEBUG_ON
    va_list va;
    va_start(va, fmt);
    char message[255];
    vsnprintf(message, sizeof(message), fmt, va);
    LHMMessage::debugSerial.println(message);
    va_end(va);
#endif
}

void LHMMessage::sendCommandFeedback(CommandType cmd, bool isSuccessful)
{
    sendCommandFeedback((int)cmd, isSuccessful);
}

void LHMMessage::sendCommandFeedback(int cmd, bool isSuccessful)
{
    int result = isSuccessful ? 1 : 0;
    char message[8];
    snprintf(message, sizeof(message), "$%c%i,%i$", SERIAL_MESSAGE_TYPE_INDICATOR_FBK, cmd, result);
    sendDebugMessage("LHMMessage::sendCommandFeedback::%s", message);
    comSerial.println(message);
}