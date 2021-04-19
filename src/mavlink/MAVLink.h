#ifndef MIMREE_DRM_CONTROLLER_MAVLINK_H
#define MIMREE_DRM_CONTROLLER_MAVLINK_H

#define MAVLINK_CHECK_MESSAGE_LENGTH

#define MAV_DEBUG

#ifdef MAV_DEBUG
#define MAV_DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#define MAV_DEBUG_PRINTLN(a) Serial.println(a)
#else
#define MAV_DEBUG_PRINTF(fmt, ...) 
#define MAV_DEBUG_PRINTLN(a)
#endif

#include <Arduino.h>
#include <HardwareSerial.h>

#include "c_library_v2/ardupilotmega/mavlink.h"
#include "c_library_v2/checksum.h"
#include "c_library_v2/mavlink_types.h"
#include "c_library_v2/protocol.h"

const uint8_t MIMREE_UOM_MAV_TYPE = MAV_TYPE_OCTOROTOR;
const uint8_t MIMREE_UOM_AUTOPILOT_TYPE = MAV_AUTOPILOT_INVALID;
const uint8_t MAV_DATA_TO_STREAM = MAV_DATA_STREAM_ALL;
const uint16_t MAV_DATA_REQUEST_RATE_HZ = 5;
const uint32_t MAV_DEFAULT_TIME_OUT = 1000/MAV_DATA_REQUEST_RATE_HZ*3;
const mavlink_channel_t MAV_CHANNEL = MAVLINK_COMM_0;

class MAVLink
{
public:
    MAVLink(HardwareSerial &hs, uint8_t sysid, uint8_t compid);
    bool initiate(unsigned long baudrate);
    bool readMessage(mavlink_message_t *msg);
    void sendMessage(mavlink_message_t *msg);
    bool waitMessage(mavlink_message_t *msg, uint32_t msgId, uint32_t timeoutMS=MAV_DEFAULT_TIME_OUT);
    void sendHeartbeat();
    bool readHeartbeat();
    void requestStream();
    void setMessageInterval(uint16_t msgid, uint32_t interval);
    inline void setDebugVectInterval(uint32_t interval) { setMessageInterval(MAVLINK_MSG_ID_DEBUG_VECT, interval);}
    bool confirmHandshake(uint32_t timeoutMS=MAV_DEFAULT_TIME_OUT);
    void sendDebug(uint32_t timestamp, uint8_t index, float value);
    void sendDebugVect(char &name, uint32_t timestamp, float x, float y, float z);
    mavlink_debug_t unpackMessageToDebug(mavlink_message_t *msg);
    mavlink_debug_vect_t unpackMessageToDebugVect(mavlink_message_t *msg);
    void sendCommandAck(uint16_t cmdId, uint8_t result, uint8_t progress, uint8_t cmdParam, uint8_t targetSysid, uint8_t targetCompid);
    void sendTakeOffCommand();

private:
    HardwareSerial *mavSerial;
    uint8_t pixhawk_sysid;
    uint8_t pixhawk_compid;
    uint8_t sysid;
    uint8_t compid;
};

#endif
