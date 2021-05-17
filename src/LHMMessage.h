#ifndef MIMREE_DRM_CONTROLLER_LHM_MESSAGE_H
#define MIMREE_DRM_CONTROLLER_LHM_MESSAGE_H

#define __builtin_va_start(v, l)
#define __builtin_va_end(v)

#include <Arduino.h>
#include <USBSerial.h>

#include "LHMDataTable.h"
#include "LHMController.h"
#include "utilities/LEDController.h"
#include "mavlink/MAVLink.h"
#include "mavlink/c_library_v2/mavlink_types.h"

struct MAVMessage
{
    inline MAVMessage(uint32_t msgId, bool accepted) : msgId(msgId), accepted(accepted)
    {
    }
    virtual ~MAVMessage() {}

    uint32_t msgId;
    bool accepted;

    // mavlink_command_int_t command_int;
    // mavlink_command_ack_t command_ack;
    mavlink_button_change_t button_change;
};

// struct MAVCommandIntMessage : public MAVMessage
// {
//     inline MAVCommandIntMessage(mavlink_command_int_t command, bool accepted) : MAVMessage(MAVLINK_MSG_ID_COMMAND_INT, accepted)
//     {
//         this->command_int = command;
//     }
// };

// struct MAVCommandAckMessage : public MAVMessage
// {
//     inline MAVCommandAckMessage(mavlink_command_ack_t ack, bool accepted) : MAVMessage(MAVLINK_MSG_ID_COMMAND_ACK, accepted)
//     {
//         this->command_ack = ack;
//     }
// };

struct MAVButtonChangeMessage : public MAVMessage
{
    inline MAVButtonChangeMessage(mavlink_button_change_t bc, bool accepted) : MAVMessage(MAVLINK_MSG_ID_BUTTON_CHANGE, accepted)
    {
        this-> button_change = bc;
    }
};

class LHMMessage
{
public:
    LHMMessage(LHMController &controller);

    inline bool initiate()
    {
        if (!mavlink.initiate(LHM_MAV_BAUDRATE))
        {
            return false;
        }
        // mavlink.requestStream(255U, 0U);
        // mavlink.requestStream(1U, 1U);
        return true;
    }

    MAVMessage readMAVMessage();
    MAVButtonChangeMessage readButtonChangeMessage(mavlink_message_t *msg);
    void sendCommandFeedback(uint8_t cmd, bool result, uint8_t progress);
#ifdef COMM_LINK_TEST
    void sendStatusMessage(lhm_hinge_status_t hingeStatus, uint32_t hookStatus, uint8_t payload);
#else
    void sendStatusMessage(lhm_hinge_status_t hingeStatus, lhm_hook_status_t hookStatus, uint8_t payload);
#endif

    MAVLink mavlink;

private:
    LHMController &lhmController;
    uint32_t msgIdHist[256];
    uint8_t msgIdHistPt = 0;

    inline void addToMsgIdHist(uint32_t msgid)
    {
        msgIdHist[msgIdHistPt] = msgid;
        msgIdHistPt++;
    }

    inline void clearMsgIdHist()
    {
        memset(msgIdHist, 0, 255);
        msgIdHistPt = 0;
    }

    inline String msgIdHistToString()
    {
        String msgids = "";
        for (int i = 0; i < msgIdHistPt; i++)
        {
            char c[4];
            itoa(msgIdHist[i], c, 10);
            msgids += c;
            msgids += ",";
        }
        return msgids;
    }

    inline void blinkCOMLED() { lhmController.ledRed.flash(1, FLASH_TIME_BLINK, 1); }
};

#endif