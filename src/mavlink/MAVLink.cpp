#include "MAVLink.h"

MAVLink::MAVLink(HardwareSerial &hs, uint8_t _sysid, uint8_t _compid)
{
    _MAVSerial = &hs;
    sysid = _sysid;
    compid = _compid;
}

bool MAVLink::initiate(unsigned long baudrate)
{
    _MAVSerial->begin(baudrate);
    delay(100);
    bool result = confirmHeartbeat(2000U);
    if (!result)
    {
        Serial.println("MAVLink::initiate: Fail to initiate COM.");
        return false;
    }
    Serial.println("MAVLink::initiate: COM initiated.");
    return true;
}

bool MAVLink::readMessage(mavlink_message_t *msg)
{
    mavlink_status_t status;
    while (_MAVSerial->available() > 0)
    {
        uint8_t ch = _MAVSerial->read();

        if (mavlink_parse_char(MAV_CHANNEL, ch, msg, &status))
        {
            MAV_DEBUG_PRINTF("MAVLink::readMessage: [C] id=[%d] seq=[%d] sid=%d cid=%d | STATUS: mr=%d, bo=%d, pe=%d, ps=%d, pi=%d, crs=%d, cts=%d, prsc=%d, prdc=%d, f=%d, sw=%d\n", msg->msgid, msg->seq, msg->sysid, msg->compid, status.msg_received, status.buffer_overrun, status.parse_error, (int)status.parse_state, status.packet_idx, status.current_rx_seq, status.current_tx_seq, status.packet_rx_success_count, status.packet_rx_drop_count, status.flags, status.signature_wait);
            return true;
        }
        MAV_DEBUG_PRINTF("MAVLink::readMessage: [U] id=[%d] seq=[%d] sid=%d cid=%d | STATUS: mr=%d, bo=%d, pe=%d, ps=%d, pi=%d, crs=%d, cts=%d, prsc=%d, prdc=%d, f=%d, sw=%d\n", msg->msgid, msg->seq, msg->sysid, msg->compid, status.msg_received, status.buffer_overrun, status.parse_error, (int)status.parse_state, status.packet_idx, status.current_rx_seq, status.current_tx_seq, status.packet_rx_success_count, status.packet_rx_drop_count, status.flags, status.signature_wait);
    }
    MAV_DEBUG_PRINTLN("MAVLink::readMessage: Serial buffer ends with none or incomplete message.");
    return false;
}

void MAVLink::sendMessage(mavlink_message_t *msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t bufferLen = mavlink_msg_to_send_buffer(buffer, msg);
    // delay(1000);//why?
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
                return true;
            }
        }
        delay(1);
    }
    return false;
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
    return true;
}

void MAVLink::requestStream()
{
    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(sysid, compid, &msg, pixhawk_sysid, pixhawk_compid, MAV_DATA_TO_STREAM, MAV_DATA_REQUEST_RATE_HZ, 1);
    sendMessage(&msg);
    Serial.println("Request sent! Now you are ready to recieve data...");
}

void MAVLink::sendHeartbeat()
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(sysid, compid, &msg, MIMREE_UOM_MAV_TYPE, MIMREE_UOM_AUTOPILOT_TYPE, MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);
    sendMessage(&msg);
}

void MAVLink::sendDebug(uint32_t timestamp, uint8_t index, float value)
{
    mavlink_message_t msg;
    mavlink_msg_debug_pack(sysid, compid, &msg, timestamp, index, value);
    sendMessage(&msg);
}

mavlink_debug_t MAVLink::unpackMessageToDebug(mavlink_message_t *msg)
{
    mavlink_debug_t data;
    if (msg->msgid == MAVLINK_MSG_ID_DEBUG)
    {
        mavlink_msg_debug_decode(msg, &data);
    }
    return data;
}

void MAVLink::sendDebugVect(char &name, uint32_t timestamp, float x, float y, float z)
{
    mavlink_message_t msg;
    mavlink_msg_debug_vect_pack(sysid, compid, &msg, &name, timestamp, x, y, z);
    sendMessage(&msg);
}

mavlink_debug_vect_t MAVLink::unpackMessageToDebugVect(mavlink_message_t *msg)
{
    mavlink_debug_vect_t data;
    if (msg->msgid == MAVLINK_MSG_ID_DEBUG_VECT)
    {
        mavlink_msg_debug_vect_decode(msg, &data);
    }
    return data;
}