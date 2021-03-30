#include "MAVLink.h"

MAVLink::MAVLink(HardwareSerial &hs)
{
    _MAVSerial = &hs;
}

bool MAVLink::initiate()
{
    _MAVSerial->begin(57600);
    delay(1000);
    if (_MAVSerial->available() <= 0)
    {
        return 0;
    }
    else
    {
        return confirmHeartbeat(2000U);
    }   
}

bool MAVLink::readMessage(mavlink_message_t *msg)
{
    if (_MAVSerial->available() > 0)
    {
        mavlink_status_t status;
        uint8_t ch = _MAVSerial->read();
        mavlink_parse_char(MAV_CHANNEL, ch, msg, &status);
        printf("MAVLink::readMessage: Message[%d] received.", msg->msgid);
        return true;
    }
    return false;
}

void MAVLink::sendMessage(mavlink_message_t *msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t bufferLen = mavlink_msg_to_send_buffer(buffer, msg);
    delay(1000);//why?
    _MAVSerial->write(buffer, bufferLen);
}

bool MAVLink::waitMessage(mavlink_message_t *msg, uint32_t msgId, uint32_t timeoutMS)
{
    uint32_t sTime = millis();
    while (millis() - sTime < timeoutMS)
    {
        if (readMessage(msg))
        {
            if (msg->msgid == msgId)
            {
                break;
            }
        }
        delay(1);
    }
}

bool MAVLink::confirmHeartbeat(uint32_t timeoutMS)
{
    sendHeartbeat();
    Serial.println("Heartbeats sent!");

    mavlink_message_t msg;
    mavlink_status_t status;
    uint32_t sTime = millis();
    if (!waitMessage(&msg, MAVLINK_MSG_ID_HEARTBEAT, timeoutMS))
    {
        Serial.println("Timeout reading Heartbeat from Pixhawk. Please retry.");
        return false;
    }
    mavlink_heartbeat_t packet;
    mavlink_msg_heartbeat_decode(&msg, &packet);
    pixhawk_sysid = msg.sysid;
    pixhawk_compid = msg.compid;
    Serial.printf("Pixhawk sysid %d, comid %d  \n", pixhawk_sysid, pixhawk_compid);
}

void MAVLink::requestStream()
{
    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, pixhawk_sysid, pixhawk_compid, MAV_DATA_TO_STREAM, MAV_DATA_REQUEST_RATE_HZ, 1);
    sendMessage(&msg);
    Serial.println("Request sent! Now you are ready to recieve data...");
}

void MAVLink::sendHeartbeat()
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_TYPE, AUTOPILOT_TYPE, MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);
    sendMessage(&msg);
}

void MAVLink::sendDebug(uint32_t timestamp, uint8_t index, float value)
{
    mavlink_message_t msg;
    mavlink_msg_debug_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, timestamp, index, value);
    sendMessage(&msg);
}

void MAVLink::readDebug(uint32_t &timestamp, uint8_t &index, float &value)
{
    while (_MAVSerial->available() > 0)
    {
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t ch = _MAVSerial->read();
        if (mavlink_parse_char(MAVLINK_COMM_0, ch, &msg, &status))
        {
            switch (msg.msgid)
            {
            case MAVLINK_MSG_ID_DEBUG:
            {
                mavlink_debug_t data;
                mavlink_msg_debug_decode(&msg, &data);
                timestamp = data.time_boot_ms;
                index = data.ind;
                value = data.value;
                break;
            }
            // case MAVLINK_MSG_ID_DEBUG_VECT:
            // {

            // }
            }
        }
    }
}


void MAVLink::sendDebugVect(char &name, uint32_t timestamp, float x, float y, float z)
{
    mavlink_message_t msg;
    mavlink_msg_debug_vect_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, &name, timestamp, x, y, z);
    sendMessage(&msg);
}