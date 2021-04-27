#include "OlamController.h"
OLAMController::OLAMController()
    : dxl(Dynamixel2Arduino(DXL_SERIAL, PIN_DXL_DIR)),
      ltcMotor(DXLMotor(dxl, 1)),
      ltController(LineTensionController(ltcMotor)),
      homeButton(17),
      engagementButton(16)
{
}

void OLAMController::setup()
{
    dxl.begin(DXL_BAUD_RATE);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    pinMode(TC_PIN_LIMIT_SWITCH_LOW_NO, INPUT_PULLUP);
    pinMode(TC_PIN_LIMIT_SWITCH_LOW_NC, INPUT_PULLUP);
    pinMode(TC_PIN_LIMIT_SWITCH_HIGH_NC, INPUT_PULLUP);
    pinMode(TC_PIN_LIMIT_SWITCH_HIGH_NO, INPUT_PULLUP);

    setupOnBoardDevices();
}

bool OLAMController::initiate()
{
    bool result = ltcMotor.reboot();
    result = result && ltcMotor.setVelocityLimit(TC_VELOCITY_MAX);
    DEBUG_SERIAL.println(result ? "OLAMController::initiate: DXL servos initiated." : "LHMController::initiateDXL: Fail to initiated DXL servos.");
}