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
        if (msg.sysid == GCS_MAV_SYS_ID && msg.compid == GCS_MAV_COMP_ID)
        {
            DEBUG_SERIAL.println("LHMMessage::readMAVMessage: [I] Heartbeat from GCS.");
            // mavlink_heartbeat_t heartbeat;
            // mavlink_msg_heartbeat_decode(&msg, &heartbeat);
            // DEBUG_SERIAL.printf("LHMMessage::readMAVMessage: [I] Heartbeat [%d-%d] | cm=%d, t=%d, ap=%d, bm=%d, ss=%d, mver=%d.\n", msg.sysid, msg.compid, heartbeat.custom_mode, heartbeat.type, heartbeat.autopilot, heartbeat.base_mode, heartbeat.system_status, heartbeat.mavlink_version);
        }
        if (msgIdHistPt>0)
        {
            DEBUG_SERIAL.printf("Ignored messages: %s\n", msgIdHistToString().c_str());
            clearMsgIdHist();
        }
        return MAVMessage(MAVLINK_MSG_ID_HEARTBEAT, true);
    }

#ifdef MAV_DEBUG_ATTITUDE
    if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE)
    {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&msg, &attitude);
        DEBUG_SERIAL.printf("LHMMessage::readMAVMessage: [I] attitude [seq:%d] from [%d-%d]: @%d (r=%.3f p=%.3f y=%.3f)\n", msg.seq, msg.sysid, msg.compid, attitude.time_boot_ms, attitude.pitch, attitude.roll, attitude.yaw);
        return MAVMessage(MAVLINK_MSG_ID_ATTITUDE, true);
    }
#endif

    // if (msg.msgid == MAVLINK_MSG_ID_DEBUG_VECT)
    // {
    //     mavlink_debug_vect_t data = mavlink.unpackMessageToDebugVect(&msg);
    //     DEBUG_SERIAL.printf("LHMMessage::readMAVMessage: [I] debug_vect [seq:%d] from [%d-%d]: %s @%d (%.1f, %.1f, %.1f)\n", msg.seq, msg.sysid, msg.compid, data.name, data.time_usec, data.x, data.y, data.z);
    //     return MAVMessage(MAVLINK_MSG_ID_DEBUG_VECT, true);
    // }

    // if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK)
    // {
    //     DEBUG_SERIAL.printf("ack");
    //     mavlink_command_ack_t ack;
    //     mavlink_msg_command_ack_decode(&msg, &ack);
    //     if (ack.target_component != 0 && ack.target_component != LHM_MAV_COMP_ID)
    //     {
    //         DEBUG_SERIAL.printf("LHMMessage::readMAVMessage: [I][NFU] command_ack [seq:%d] [cmd:%d re:%d prog:%d] from [%d-%d] to [%d-%d]\n", msg.seq, ack.command, ack.result, ack.progress, msg.sysid, msg.compid, ack.target_system, ack.target_component);
    //         return MAVCommandAckMessage(ack, false);
    //     }
    //     DEBUG_SERIAL.printf("LHMMessage::readMAVMessage: [I] command_ack [seq:%d] [cmd:%d re:%d prog:%d] from [%d-%d] to [%d-%d]\n", msg.seq, ack.command, ack.result, ack.progress, msg.sysid, msg.compid, ack.target_system, ack.target_component);
    //     return MAVCommandAckMessage(ack, true);
    // }

    // if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT)
    // {
    //     mavlink_command_int_t command;
    //     mavlink_msg_command_int_decode(&msg, &command);
    //     DEBUG_SERIAL.printf("LHMMessage::readMAVMessage: [I][NFU] command_int [seq:%d] [cmd:%d] from [%d-%d] to [%d-%d]\n", msg.seq, command.command, msg.sysid, msg.compid, command.target_system, command.target_component);
            
    //     if (command.target_component != 0 && command.target_component != LHM_MAV_COMP_ID)
    //     {
    //         DEBUG_SERIAL.printf("LHMMessage::readMAVMessage: [I][NFU] command_int [seq:%d] [cmd:%d] from [%d-%d] to [%d-%d]\n", msg.seq, command.command, msg.sysid, msg.compid, command.target_system, command.target_component);
    //         return MAVCommandIntMessage(command, false);
    //     }
    //     bool accepted = false;
    //     if (command.command == MAV_CMD_DO_SET_MODE)
    //     {
    //         accepted = true;
    //     }
    //     DEBUG_SERIAL.printf("LHMMessage::readMAVMessage: [I][%s] command_int [seq:%d] [cmd:%d] from [%d-%d] to [%d-%d] (params: %d)\n", accepted ? 'A' : 'R', msg.seq, command.command, msg.sysid, msg.compid, command.target_system, command.target_component, command.param1, command.param2, command.param3, command.param4);
    //     return MAVCommandIntMessage(command, accepted);
    // }
    
    // if (msg.msgid == MAVLINK_MSG_ID_PROTOCOL_VERSION)
    // {
    //     mavlink_protocol_version_t pv;
    //     mavlink_msg_protocol_version_decode(&msg, &pv);
    //     DEBUG_SERIAL.printf("LHMMessage::readMAVMessage: [I] protocol_version [seq:%d] from [%d-%d] (ver=[%d])\n", msg.seq, msg.sysid, msg.compid, pv.version);
    //     return MAVMessage(LHM_MAV_MSG_ID_PROTOCOL_VERSION, true);
    // }

    if (msg.msgid == MAVLINK_MSG_ID_BUTTON_CHANGE)
    {
        return readButtonChangeMessage(&msg);
    }

    addToMsgIdHist(msg.msgid);
    return MAVMessage(LHM_MAV_MSG_ID_NOT_FOR_US, false);
}

MAVButtonChangeMessage LHMMessage::readButtonChangeMessage(mavlink_message_t* msg)
{
    mavlink_button_change_t bc;
    if (msg->sysid == GCS_MAV_SYS_ID && msg->compid == GCS_MAV_SYS_ID)
    {
        DEBUG_SERIAL.printf("LHMMessage::readMAVMessage: [I][NFU] button_change [seq:%d] from [%d-%d]\n", msg->seq, msg->sysid, msg->compid);
        return MAVButtonChangeMessage(bc, false);
    }
    mavlink_msg_button_change_decode(msg, &bc);
    if (bc.last_change_ms == LHM_MAV_MSG_BUTTON_CHANGE_PASSCODE)
    {
        DEBUG_SERIAL.printf("LHMMessage::readMAVMessage: [I] button_change [seq:%d] from [%d-%d] (s=[%d])\n", msg->seq, msg->sysid, msg->compid, bc.state);
    }
    return MAVButtonChangeMessage(bc, true);
}

void LHMMessage::sendCommandFeedback(uint16_t cmd, bool result, uint8_t progress)
{
    if (cmd == LHM_CMD_ID_UNKNOWN || progress== LHM_CMD_PROG_COMMAND_ACK)
    {
        return;
    }
    mavlink.sendCommandAck(cmd, result, progress, 0, LHM_MAV_SYS_ID, GCS_MAV_COMP_ID);
}

void LHMMessage::sendStatusMessage(lhm_hinge_status_t hingeStatus, lhm_hook_status_t hookStatus, uint8_t payload)
{
    char name[10];
    strcpy(name, LHM_MAV_MSG_DEBUG_VECT_NAME_STATUS_REPORT);
    uint32_t timestamp = 0;
    mavlink.sendDebugVect(*name, timestamp, (float)hingeStatus, (float)hookStatus, (float)payload);
}