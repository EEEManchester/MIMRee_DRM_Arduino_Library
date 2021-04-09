#include "LHMMessage.h"

LHMMessage::LHMMessage(LHMController &controller) : mavlink(MAVLink(COM_SERIAL, LHM_MAV_SYS_ID, LHM_MAV_COMP_ID)), lhmController(controller)
{
}

MAVMessage LHMMessage::readMAVMessage()
{
    mavlink_message_t msg;
    if (!mavlink.readMessage(&msg))
    {
        return MAVMessage(LHM_MAV_MSG_ID_NOT_FOR_US, false);
    }

    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
    {
        blinkCOMLED();
        DEBUG_SERIAL.printf("LHMMessage::readMAVMessage: [I] Heartbeat received from [%d-%d]. Ignored messages: %s\n", msg.sysid, msg.compid, msgIdHistToString().c_str());
        clearMsgIdHist();
        return MAVMessage(LHM_MAV_MSG_ID_HEARTBEAT, true);
    }
#ifdef MAV_DEBUG_ATTITUDE
    if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE)
    {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&msg, &attitude);
        DEBUG_SERIAL.printf("LHMMessage::readMAVMessage: [I] attitude [seq:%d] from [%d-%d]: @%d (r=%.3f p=%.3f y=%.3f)\n", msg.seq, msg.sysid, msg.compid, attitude.time_boot_ms, attitude.pitch, attitude.roll, attitude.yaw);
        return MAVMessage(LHM_MAV_MSG_ID_ATTITUDE, true);
    }
#endif

    if (msg.msgid == MAVLINK_MSG_ID_DEBUG_VECT)
    {
        mavlink_debug_vect_t data = mavlink.unpackMessageToDebugVect(&msg);
        DEBUG_SERIAL.printf("LHMMessage::readMAVMessage: [I] debug_vect [seq:%d] from [%d-%d]: %s @%d (%.1f, %.1f, %.1f)\n", msg.seq, msg.sysid, msg.compid, data.name, data.time_usec, data.x, data.y, data.z);
        return MAVMessage(LHM_MAV_MSG_ID_DEBUG_VECT, true);
    }

    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK)
    {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(&msg, &ack);
        if (ack.target_component != LHM_MAV_COMP_ID)
        {
            MAV_DEBUG_PRINTF("LHMMessage::readMAVMessage: [I][NFU] command_ack [seq:%d] from [%d-%d] to [%d-%d]\n", msg.seq, msg.sysid, msg.compid, ack.target_system, ack.target_component);
            return MAVCommandAckMessage(ack, false);
        }
        DEBUG_SERIAL.printf("LHMMessage::readMAVMessage: [I] command_ack [seq:%d] [cmd:%d re:%d prog:%d] from [%d-%d] to [%d-%d]\n", msg.seq, ack.command, ack.result, ack.progress, msg.sysid, msg.compid, ack.target_system, ack.target_component);
        return MAVCommandAckMessage(ack, true);
    }

    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT)
    {
        mavlink_command_int_t command;
        mavlink_msg_command_int_decode(&msg, &command);
        if (command.target_component != LHM_MAV_COMP_ID)
        {
            MAV_DEBUG_PRINTF("LHMMessage::readMAVMessage: [I][NFU] command_int [seq:%d] [cmd:%d] from [%d-%d] to [%d-%d]\n", msg.seq, command.command, msg.sysid, msg.compid, ack.target_system, ack.target_component);
            return MAVCommandIntMessage(command, false);
        }
        bool accepted = false;
        if (command.command == MAV_CMD_DO_SET_MODE)
        {
            accepted = true;
        }
        DEBUG_SERIAL.printf("LHMMessage::readMAVMessage: [I][%s] command_int [seq:%d] [cmd:%d] from [%d-%d] to [%d-%d] (param: %d)\n", accepted ? 'A' : 'R', msg.seq, command.command, msg.sysid, msg.compid, command.target_system, command.target_component, command.param1, command.param2, command.param3, command.param4);
        return MAVCommandIntMessage(command, accepted);
    }

    addToMsgIdHist(msg.msgid);
    return MAVMessage(LHM_MAV_MSG_ID_NOT_FOR_US, false);
}

void LHMMessage::sendCommandFeedback(uint16_t cmd, bool result, uint8_t progress)
{
    if (cmd < 0)
    {
        return;
    }
    mavlink.sendCommandAck(cmd, result, progress, 0, LHM_MAV_SYS_ID, GCS_MAV_COMP_ID);
}

void LHMMessage::sendStatusMessage(lhm_hinge_status_t hingeStatus, lhm_hook_status_t hookStatus, uint8_t payload)
{
    char name[10];
    strcpy(name, LHM_MSG_TYPE_STATUS_REPORT);
    uint32_t timestamp = 0;
    mavlink.sendDebugVect(*name, timestamp, (float)hingeStatus, (float)hookStatus, (float)payload);
}