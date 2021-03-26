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
long hookMotionStopDelay = 100;
long hookMotionStartTime = 0;
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

  if (hookMotionStartTime > 0 && ((int)millis() - hookMotionStartTime) > hookMotionStopDelay)
  {
    HookStatus hkMotion = lhm.getHookMotionStatus();
    if (hkMotion == HookStatus::OPENNING || hkMotion == HookStatus::CLOSING)
    {
      hkStatus = lhm.getHookStatus();
      DEBUG_SERIAL.printf("Status_rapid: Hook=%d\n", (int)hkStatus);
      if (hkStatus != HookStatus::OPENNING && hkStatus != HookStatus::CLOSING)
        if (lhm.stopHookMotor())
        {
          hookMotionStartTime = 0;
        }
    }
  }

  LSButtonState btnJetState = lhm.btnJet.getState();
  if (btnJetState == LSButtonState::SHORT_PRESSED)
  {
    DEBUG_SERIAL.println("On-board button pressed, requesting locking LHM.");
    lhm.lockLHM();
  }
  else if (btnJetState == LSButtonState::LONG_PRESSED)
  {
    DEBUG_SERIAL.println("On-board button pressed, requesting releasing LHM.");
    lhm.jettison();
  }
}

void reportStatus()
{
  hgStatus = lhm.getHingeStatus();
  hkStatus = lhm.getHookStatus();
  if (hgStatus == HingeStatus::ERROR || hgStatus == HingeStatus::OFFLINE || hgStatus == HingeStatus::UNKNOWN)
  {
    digitalWrite(PIN_LED_BLUE, HIGH);
  }
  else
  {
    digitalWrite(PIN_LED_BLUE, LOW);
  }
  if (hkStatus == HookStatus::ERROR || hkStatus == HookStatus::OFFLINE || hkStatus == HookStatus::UNKNOWN)
  {
    digitalWrite(PIN_LED_GREEN, HIGH);
  }
  else
  {
    digitalWrite(PIN_LED_GREEN, LOW);
  }
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
    hookMotionStartTime = millis();
    break;
  case (int)CommandType::HOOK_OPEN:
    lhm.openHook();
    hookMotionStartTime = millis();
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
