#include "OlamController.h"
OLAMController::OLAMController()
    : dxl(Dynamixel2Arduino(DXL_SERIAL, PIN_DXL_DIR)),
      ltcMotor(DXLMotor(dxl, 1, DEBUG_SERIAL)),
      ltController(LineTensionController(ltcMotor))
{
}

void OLAMController::initiate()
{
    dxl.begin(DXL_BAUD_RATE);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    ltcMotor.reboot();

    pinMode(TC_PIN_LIMIT_SWITCH_LOW_NO, INPUT_PULLUP);
    pinMode(TC_PIN_LIMIT_SWITCH_LOW_NC, INPUT_PULLUP);
    pinMode(TC_PIN_LIMIT_SWITCH_HIGH_NC, INPUT_PULLUP);
    pinMode(TC_PIN_LIMIT_SWITCH_HIGH_NO, INPUT_PULLUP);
}