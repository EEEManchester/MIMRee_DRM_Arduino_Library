#ifndef MIMREE_DRM_CONTROLLER_LHM_MESSAGE_H
#define MIMREE_DRM_CONTROLLER_LHM_MESSAGE_H

#define __builtin_va_start(v, l)
#define __builtin_va_end(v)

#define LHM_DEBUG_ON 1

#include <Arduino.h>
#include <USBSerial.h>

#include "LHMDataTable.h"
#include "LHMController.h"
#include "utilities/LEDController.h"
#include "mavlink/MAVLink.h"
#include "mavlink/c_library_v2/mavlink_types.h"

class LHMMessage
{
public:
    LHMMessage(LHMController &controller);
    inline bool initiate() { return mavlink.initiate(LHM_MAV_BAUDRATE); }
    int32_t readCommandIn();
    void sendCommandExecutionFeedback(int32_t cmd, bool isSuccessful);
    void sendCommandFeedbackReception(int32_t cmd, bool isSuccessful);
    void sendStatusMessage(HingeStatus hingeStatus, HookStatus hookStatus, uint8_t payload);

private:
    MAVLink mavlink;
    LHMController &lhmController;
    uint32_t msgIdHist[256];
    uint8_t msgIdHistPt = 0;

    inline void addToMsgIdHist(uint32_t msgid)
    {
        msgIdHist[msgIdHistPt] = msgid;
        msgIdHistPt ++;
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