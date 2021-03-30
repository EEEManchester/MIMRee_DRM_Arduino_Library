#ifndef MIMREE_DRM_CONTROLLER_MAVLINK_H
#define MIMREE_DRM_CONTROLLER_MAVLINK_H

#include <Arduino.h>
#include <HardwareSerial.h>

#include "c_library_v2/ardupilotmega/mavlink.h"
#include "c_library_v2/checksum.h"
#include "c_library_v2/mavlink_types.h"
#include "c_library_v2/protocol.h"

const uint8_t MAV_SYSTEM_ID = 1;
const uint8_t MAV_COMPONENT_ID = 158;
const uint8_t MAV_TYPE = MAV_TYPE_QUADROTOR;
const uint8_t AUTOPILOT_TYPE = MAV_AUTOPILOT_INVALID;
const uint8_t MAV_DATA_TO_STREAM = MAV_DATA_STREAM_ALL;
const uint16_t MAV_DATA_REQUEST_RATE_HZ = 5;
const uint32_t MAV_DEFAULT_TIME_OUT = 1000/MAV_DATA_REQUEST_RATE_HZ*3;
const mavlink_channel_t MAV_CHANNEL = MAVLINK_COMM_0;

class MAVLink
{
public:
    MAVLink(HardwareSerial &hs);
    bool initiate();
    bool readMessage(mavlink_message_t *msg);
    void sendMessage(mavlink_message_t *msg);
    bool waitMessage(mavlink_message_t *msg, uint32_t msgId, uint32_t timeoutMS=MAV_DEFAULT_TIME_OUT);
    void sendHeartbeat();
    bool readHeartbeat();
    void requestStream();
    bool confirmHeartbeat(uint32_t timeoutMS=MAV_DEFAULT_TIME_OUT);
    void sendDebug(uint32_t timestamp, uint8_t index, float value);
    void readDebug(uint32_t &timestamp, uint8_t &index, float &value);
    void sendDebugVect(char &name, uint32_t timestamp, float x, float y, float z);
    void readDebugVect(char &name, uint32_t &timestamp, uint8_t &index, float &value);

private:
    HardwareSerial *_MAVSerial;
    uint8_t pixhawk_sysid;
    uint8_t pixhawk_compid;
};

#endif
