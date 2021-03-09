#include <LinkHookModuleController.h>

#define DXL_SERIAL   Serial3
#define COM_SERIAL   Serial
#define DEBUG_SERIAL Serial

const int STATUS_REPORT_INTERVAL = 1000;
const int COMMAND_IN__CHECK_INTERVAL = 0;

LHMController lhm(DXL_SERIAL, COM_SERIAL, DEBUG_SERIAL);

void setup() {
  DEBUG_SERIAL.begin(9600);
  lhm.initiate();
  pulseLEDSequantial();
}

long prevStatusReportTime = 0;
long prevCommandInCheckTime = 0;


void loop() {
  if ((int)millis() - prevStatusReportTime > STATUS_REPORT_INTERVAL)
  {
    prevStatusReportTime = millis();
    reportStatus();
  }
  if ((int)millis() - prevCommandInCheckTime > COMMAND_IN__CHECK_INTERVAL)
  {
    prevCommandInCheckTime = millis();
    checkCommandIn();
  }
}

int reportStatus()
{
  HingeStatus hgStatus = lhm.getHingeStatus();
  HookStatus hkStatus = lhm.getHookStatus();
  DEBUG_SERIAL.printf("Status: Hinge=%d | Hook=%d\n", (int)hgStatus, (int)hkStatus);
  COM_SERIAL.printf("$S%d,%d$\n", (int)hgStatus, (int)hkStatus);
}

void checkCommandIn()
{
  int cmd = lhm.lhmMessage.readCommandIn();
  switch (cmd)
  {
  case (int)CommandType::ERROR:
    break;
  case (int)CommandType::EMERGENCY_JETTISON:
    break;
  case (int)CommandType::RESET_DYNAMIXEL_COM:
    lhm.initiate();
    break;
  case (int)CommandType::HINGE_POWER_OFF:
    lhm.stopHingeMotor();
    break;
  case (int)CommandType::HINGE_TAKE_OFF:
    lhm.setTakeoffMode();
    break;
  case (int)CommandType::HINGE_LANDING:
    lhm.setLandingPosition();
    break;
  case (int)CommandType::HINGE_SWING_REDUCTION:
    lhm.setSwingReductionMode();
    break;
  case (int)CommandType::HOOK_POWER_OFF:
    lhm.stopHookMotor();
    break;
  case (int)CommandType::HOOK_CLOSE:
    lhm.closeHook();
    break;
  case (int)CommandType::HOOK_OPEN:
    lhm.openHook();
    break;
  }
}

void pulseLEDSequantial() {
  lhm.hingeMotorX.flashLED(10, 200);
  lhm.hingeMotorY.flashLED(10, 200);
  lhm.hookMotor.flashLED(10, 200);
}
