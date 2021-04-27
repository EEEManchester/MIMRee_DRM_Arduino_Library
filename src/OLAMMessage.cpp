#include "OLAMMessage.h"

OLAMMessage::OLAMMessage(LHMController &controller) : mavlink(MAVLink(COM_SERIAL, LHM_MAV_SYS_ID, LHM_MAV_COMP_ID)), lhmController(controller)
{
}

MAVMessage OLAMMessage::readMAVMessage()
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
            DEBUG_SERIAL.println("OLAMMessage::readMAVMessage: [I] Heartbeat from GCS.");
            // mavlink_heartbeat_t heartbeat;
            // mavlink_msg_heartbeat_decode(&msg, &heartbeat);
            // DEBUG_SERIAL.printf("OLAMMessage::readMAVMessage: [I] Heartbeat [%d-%d] | cm=%d, t=%d, ap=%d, bm=%d, ss=%d, mver=%d.\n", msg.sysid, msg.compid, heartbeat.custom_mode, heartbeat.type, heartbeat.autopilot, heartbeat.base_mode, heartbeat.system_status, heartbeat.mavlink_version);
        }
        if (msgIdHistPt>0)
        {
            DEBUG_SERIAL.printf("Ignored messages: %s\n", msgIdHistToString().c_str());
            clearMsgIdHist();
        }
        return MAVMessage(MAVLINK_MSG_ID_HEARTBEAT, true);
    }

    if (msg.msgid == MAVLINK_MSG_ID_BUTTON_CHANGE)
    {
            return readButtonChangeMessage(&msg);
    }

    addToMsgIdHist(msg.msgid);
    return MAVMessage(LHM_MAV_MSG_ID_NOT_FOR_US, false);
}

MAVButtonChangeMessage OLAMMessage::readButtonChangeMessage(mavlink_message_t* msg)
{
    mavlink_button_change_t bc;
    if (msg->sysid != GCS_MAV_SYS_ID || msg->compid != GCS_MAV_SYS_ID)
    {
        DEBUG_SERIAL.printf("OLAMMessage::readMAVMessage: [I][NFU] button_change [seq:%d] from [%d-%d]\n", msg->seq, msg->sysid, msg->compid);
        return MAVButtonChangeMessage(bc, false);
    }
    mavlink_msg_button_change_decode(msg, &bc);
    if (bc.last_change_ms == LHM_MAV_MSG_BUTTON_CHANGE_PASSCODE)
    {
        DEBUG_SERIAL.printf("OLAMMessage::readMAVMessage: [I] button_change [seq:%d] from [%d-%d] (s=[%d])\n", msg->seq, msg->sysid, msg->compid, bc.state);
    }
    return MAVButtonChangeMessage(bc, true);
}

void OLAMMessage::sendCommandFeedback(uint16_t cmd, bool result, uint8_t progress)
{
    if (cmd == LHM_CMD_ID_UNKNOWN || progress== LHM_CMD_PROG_COMMAND_ACK)
    {
        return;
    }
    mavlink.sendCommandAck(cmd, result, progress, 0, LHM_MAV_SYS_ID, GCS_MAV_COMP_ID);
}

void OLAMMessage::sendStatusMessage(lhm_hinge_status_t hingeStatus, lhm_hook_status_t hookStatus, uint8_t payload)
{
    char name[10];
    strcpy(name, LHM_MAV_MSG_DEBUG_VECT_NAME_STATUS_REPORT);
    uint32_t timestamp = 0;
    mavlink.sendDebugVect(*name, timestamp, (float)hingeStatus, (float)hookStatus, (float)payload);
}