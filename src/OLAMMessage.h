#ifndef MIMREE_DRM_CONTROLLER_LHM_MESSAGE_H
#define MIMREE_DRM_CONTROLLER_LHM_MESSAGE_H

#define __builtin_va_start(v, l)
#define __builtin_va_end(v)

#include <Arduino.h>
#include <USBSerial.h>

#include "OlamDataTable.h"
#include "OlamController.h"
#include "LineTensionController.h"
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
    mavlink_button_change_t button_change;
};

struct MAVButtonChangeMessage : public MAVMessage
{
    inline MAVButtonChangeMessage(mavlink_button_change_t bc, bool accepted) : MAVMessage(MAVLINK_MSG_ID_BUTTON_CHANGE, accepted)
    {
        this-> button_change = bc;
    }
};

class OLAMMessage
{
public:
    OLAMMessage(OLAMController &controller);

    inline bool initiate(uint32_t mavCOMInterval)
    {
        if (!mavlink.initiate(OLAM_MAV_BAUDRATE))
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
    void sendStatusMessage(uint8_t lineStatus, uint8_t commandSeq, uint8_t commandExecutionStatus);

    MAVLink mavlink;

private:
    OLAMController &olamController;
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

    inline void blinkCOMLED() { olamController.ledRed.flash(1, FLASH_TIME_BLINK, 1); }
};

#endif