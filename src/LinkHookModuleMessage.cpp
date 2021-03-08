#include "LinkHookModuleMessage.h"

LHMMessage::LHMMessage(COM_SERIAL_CLASS &comSerial, DEBUG_SERIAL_CLASS &debugSerial) : comSerial(comSerial), debugSerial(debugSerial)
{
}

int LHMMessage::readCommandIn()
{
    String message = getSerialMessage();
    if (message == "")
        return (int)CommandType::ERROR;
    return parseSerialMessage(message);
}

String LHMMessage::getSerialMessage()
{
    if (comSerial.available() < 1)
    {
        return "";
    }
    String val = comSerial.readString();
    debugSerial.printf("LHMMessage::getSerialMessage::Incoming message received: %s\n", val.c_str());
    return val;
}

int LHMMessage::parseSerialMessage(String message)
{
    if (!message.startsWith(String(SERIAL_PREFIX)) || !message.endsWith(String(SERIAL_PREFIX)+"\n"))
    {
        debugSerial.printf("LHMMessage::parseSerialMessage::Wrong message format: %s\n", message.c_str());
        return (int)CommandType::ERROR;
    }
    if (message[1] == SERIAL_MESSAGE_TYPE_INDICATOR_CMD)
    {
        String command = message.substring(2, message.length() - 2);
        int val = command.toInt();
        debugSerial.printf("LHMMessage::parseSerialMessage::(Raw) %s -> (Extracted) %s -> (Converted) %D\n", message.c_str(), command.c_str(), val);
        return val;
    }
    debugSerial.printf("LHMMessage::parseSerialMessage::Code not recognised: %s\n", message.c_str());
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
    comSerial.println(message);
    debugSerial.printf("LHMMessage::sendCommandFeedback::%s\n", message);
}