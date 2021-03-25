#include <LHMController.h>
#include <LHMDataTable.h>

const int STATUS_REPORT_INTERVAL = 1000;
const int COMMAND_IN_CHECK_INTERVAL = 0;

LHMController lhm = LHMController();

void setup()
{
  DEBUG_SERIAL.begin(9600);
  lhm.initiate();
  // pulseLEDSequantial();
  DEBUG_SERIAL.println("Initialized.");
}

long prevStatusReportTime = 0;
long prevCommandInCheckTime = 0;
HingeStatus hgStatus;
HookStatus hkStatus;
bool hingeInTransition = false;

void loop()
{
  if ((int)millis() - prevStatusReportTime > STATUS_REPORT_INTERVAL)
  {
    prevStatusReportTime = millis();
    reportStatus();
    if (hingeInTransition)
    {
      trackHingeTransition();
    }
  }
  if ((int)millis() - prevCommandInCheckTime > COMMAND_IN_CHECK_INTERVAL)
  {
    prevCommandInCheckTime = millis();
    checkCommandIn();
  }
  HookStatus hkMotion = lhm.getHookMotionStatus();
  if (hkMotion == HookStatus::OPENNING || hkMotion == HookStatus::CLOSING)
  {
    if (hkStatus != HookStatus::OPENNING || hkStatus != HookStatus::OPENNING)
      lhm.stopHookMotor();
  }
}

void reportStatus()
{
  hgStatus = lhm.getHingeStatus();
  hkStatus = lhm.getHookStatus();
  bool egStatus = lhm.isEngaged();
  DEBUG_SERIAL.printf("Status: Hinge=%d | Hook=%d | Engaged=%d\n", (int)hgStatus, (int)hkStatus, (int)egStatus);
  COM_SERIAL.printf("$S%d,%d,%d$\n", (int)hgStatus, (int)hkStatus, (int)egStatus);
}

void checkCommandIn()
{
  int cmd = lhm.lhmMessage.readCommandIn();
  switch (cmd)
  {
  case (int)CommandType::ERROR:
    break;
  case (int)CommandType::JETTISON:
    lhm.jettison();
    break;
  case (int)CommandType::LOCK_LHM:
    lhm.lockLHM();
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
    hingeInTransition = lhm.setLandingPosition();
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

void trackHingeTransition()
{
  MotionSequenceStatusType status = lhm.getMotionSequenceStatus();
  DEBUG_SERIAL.printf("MotionSequenceStatus: %d\n", (int)status);

  switch (status)
  {
    case MotionSequenceStatusType::COMPLETED:
      hingeInTransition = false;
      break;
    case MotionSequenceStatusType::STAGE_COMPLETED:
      lhm.nextMotionSequence();
      break;
    case MotionSequenceStatusType::BUSY:
      break;
    case MotionSequenceStatusType::ERROR:
    case MotionSequenceStatusType::UNKNOWN:
      hingeInTransition = false;
      break;
  }
}

void pulseLEDSequantial()
{
  lhm.hingeMotorPitch.flashLED(10, 200);
  lhm.hingeMotorRoll.flashLED(10, 200);
  lhm.hookMotor.flashLED(10, 200);
}
