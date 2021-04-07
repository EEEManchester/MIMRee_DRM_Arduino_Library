#include "LHMMessage.h"

LHMMessage::LHMMessage(LHMController &controller) : mavlink(MAVLink(COM_SERIAL, LHM_MAV_SYS_ID, LHM_MAV_COMP_ID)), lhmController(controller)
{
}

int32_t LHMMessage::readCommandIn()
{
    mavlink_message_t msg;
    if (!mavlink.readMessage(&msg))
    {
        return (int8_t)CommandType::UNKNOWN;
    }
    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
    {
        blinkCOMLED();
        DEBUG_SERIAL.printf("LHMMessage::readCommmandIn: [I] Heartbeat received from [sid:%d cid:%d. Message ID Hist: %s\n", msg.sysid, msg.compid, msgIdHistToString().c_str());
        clearMsgIdHist();
        return (int8_t)CommandType::UNKNOWN;
    }
#ifdef MAV_DEBUG
    if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE)
    {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&msg, &attitude);
        DEBUG_SERIAL.printf("LHMMessage::readCommandIn: [I] attitude [seq:%d] from [sid:%d cid:%d]: @%d (r=%.3f p=%.3f y=%.3f)\n", msg.seq, msg.sysid, msg.compid, attitude.time_boot_ms, attitude.pitch, attitude.roll, attitude.yaw);
        clearMsgIdHist();
        return (int8_t)CommandType::UNKNOWN;
    }
#endif
    addToMsgIdHist(msg.msgid);
    if (msg.msgid != MAVLINK_MSG_ID_DEBUG_VECT)
    {
        return (int8_t)CommandType::UNKNOWN;
    }
    mavlink_debug_vect_t data = mavlink.unpackMessageToDebugVect(&msg);
    DEBUG_SERIAL.printf("LHMMessage::readCommmandIn: [I] debug_vect [seq:%d] from [sid:%d cid:%d]: %s @%d (%.1f, %.1f, %.1f)\n", msg.seq, msg.sysid, msg.compid, data.name, data.time_usec, data.x, data.y, data.z);
    if (data.name != LHM_MSG_TYPE_CMD_IN)
    {
        return (int8_t)CommandType::ERROR;
    }
    DEBUG_SERIAL.println("LHMMessage:readCommandIn: [I][C] Command received from Ground Station.");
    return (int32_t)data.x;
}

void LHMMessage::sendCommandExecutionFeedback(int32_t cmd, bool isSuccessful)
{
    char name[10];
    strcpy(name, LHM_MSG_TYPE_CMD_FB);
    uint32_t timestamp = 0;
    mavlink.sendDebugVect(*name, timestamp, (float)cmd, 1.0, (float)isSuccessful);
    DEBUG_SERIAL.printf("LHMMessage::sendCommandFeedback:: [O][F] @%d %i:%i\n", timestamp, cmd, isSuccessful);
}

void LHMMessage::sendCommandFeedbackReception(int32_t cmd, bool isSuccessful)
{
    char name[10];
    strcpy(name, LHM_MSG_TYPE_CMD_FB);
    uint32_t timestamp = 0;
    mavlink.sendDebugVect(*name, timestamp, (float)cmd, (float)isSuccessful, -1);
    DEBUG_SERIAL.printf("LHMMessage::sendCommandFeedbackReception:: [O][F] @%d %i:%i\n", timestamp, cmd, isSuccessful);
}

void LHMMessage::sendStatusMessage(HingeStatus hingeStatus, HookStatus hookStatus, uint8_t payload)
{
    char name[10];
    strcpy(name, LHM_MSG_TYPE_STATUS_REPORT);
    uint32_t timestamp = 0;
    mavlink.sendDebugVect(*name, timestamp, (float)hingeStatus, (float)hookStatus, (float)payload);
    DEBUG_SERIAL.printf("LHMMessage::sendStatusMessage:: [O] Status message sent @%d.\n", timestamp);
}