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
#ifdef MAV_DEBUG
        if (msg.sysid == GCS_MAV_SYS_ID && msg.compid == GCS_MAV_COMP_ID)
        {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&msg, &heartbeat);
            MAV_DEBUG_PRINTF("LHMMessage::readMAVMessage: [I] Heartbeat from GCS [%d-%d] | cm=%d, t=%d, ap=%d, bm=%d, ss=%d, mver=%d.\n", msg.sysid, msg.compid, heartbeat.custom_mode, heartbeat.type, heartbeat.autopilot, heartbeat.base_mode, heartbeat.system_status, heartbeat.mavlink_version);
        }
        if (msgIdHistPt > 0)
        {
            MAV_DEBUG_PRINTF("Ignored messages: %s\n", msgIdHistToString().c_str());
            clearMsgIdHist();
        }
#endif

        return MAVMessage(MAVLINK_MSG_ID_HEARTBEAT, true);
    }

    if (msg.msgid == MAVLINK_MSG_ID_BUTTON_CHANGE)
    {
        return readButtonChangeMessage(&msg);
    }

    addToMsgIdHist(msg.msgid);
    return MAVMessage(LHM_MAV_MSG_ID_NOT_FOR_US, false);
}

MAVButtonChangeMessage LHMMessage::readButtonChangeMessage(mavlink_message_t *msg)
{
    mavlink_button_change_t bc;
    if (msg->sysid != GCS_MAV_SYS_ID || msg->compid != GCS_MAV_COMP_ID)
    {
        MAV_DEBUG_PRINTF("LHMMessage::readMAVMessage: [I][NFU] button_change [seq:%d] from [%d-%d]\n", msg->seq, msg->sysid, msg->compid);
        return MAVButtonChangeMessage(bc, false);
    }
    mavlink_msg_button_change_decode(msg, &bc);
    if (bc.last_change_ms == LHM_MAV_MSG_BUTTON_CHANGE_PASSCODE)
    {
        MAV_DEBUG_PRINTF("LHMMessage::readMAVMessage: [I] button_change [seq:%d] from [%d-%d] (s=[%d])\n", msg->seq, msg->sysid, msg->compid, bc.state);
    }
    return MAVButtonChangeMessage(bc, true);
}

void LHMMessage::sendCommandFeedback(uint8_t cmd, bool result, uint8_t progress)
{
    if (cmd == LHM_CMD_ID_UNKNOWN || progress == LHM_CMD_PROG_COMMAND_ACK)
    {
        return;
    }
    mavlink.sendCommandAck(cmd, result, progress, 0, LHM_MAV_SYS_ID, GCS_MAV_COMP_ID);
}

#ifdef COMM_LINK_TEST
void LHMMessage::sendStatusMessage(lhm_hinge_status_t hingeStatus, uint32_t hookStatus, uint8_t payload)
#else
void LHMMessage::sendStatusMessage(lhm_hinge_status_t hingeStatus, lhm_hook_status_t hookStatus, uint8_t payload)
#endif
{
    char name[10];
    strcpy(name, LHM_MAV_MSG_DEBUG_VECT_NAME_STATUS_REPORT);
    uint32_t timestamp = 0;
    mavlink.sendDebugVect(*name, timestamp, (float)hingeStatus, (float)hookStatus, (float)payload);
}