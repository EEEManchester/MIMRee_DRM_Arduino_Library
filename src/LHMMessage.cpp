#include "LHMMessage.h"
int LHMMessage::readCommandIn()
{
    String message = getSerialMessage();
    if (message == "")
        return (int)CommandType::ERROR;
    return parseSerialMessage(message);
}

String LHMMessage::getSerialMessage()
{
    if (COM_SERIAL.available() < 1)
    {
        return "";
    }
    String val = COM_SERIAL.readString();
    if (val.endsWith("\n"))
    {
        val = val.substring(0, val.length()-1);
    }
    DEBUG_SERIAL.printf("LHMMessage::getSerialMessage::Incoming message received: %s\n", val.c_str());
    return val;
}

int LHMMessage::parseSerialMessage(String message)
{
    if (!message.startsWith(String(SERIAL_PREFIX)) || !message.endsWith(String(SERIAL_PREFIX)))
    {
        DEBUG_SERIAL.printf("LHMMessage::parseSerialMessage::Wrong message format: %s\n", message.c_str());
        return (int)CommandType::ERROR;
    }
    if (message[1] == SERIAL_MESSAGE_TYPE_INDICATOR_CMD)
    {
        String command = message.substring(2, message.length() - 1);
        int val = command.toInt();
        DEBUG_SERIAL.printf("LHMMessage::parseSerialMessage::(Raw) %s -> (Extracted) %s -> (Converted) %d\n", message.c_str(), command.c_str(), val);
        return val;
    }
    DEBUG_SERIAL.printf("LHMMessage::parseSerialMessage::Code not recognised: %s\n", message.c_str());
    return (int)CommandType::ERROR;
}

void LHMMessage::sendCommandFeedback(CommandType cmd, bool isSuccessful)
{
    sendCommandFeedback((int)cmd, isSuccessful);
}

void LHMMessage::sendCommandFeedback(int cmd, bool isSuccessful)
{
    int result = isSuccessful ? 1 : 0;
    char message[32];
    snprintf(message, sizeof(message), "$%c%i,%i$", SERIAL_MESSAGE_TYPE_INDICATOR_FBK, cmd, result);
    COM_SERIAL.println(message);
    DEBUG_SERIAL.printf("LHMMessage::sendCommandFeedback::%s\n", message);
}