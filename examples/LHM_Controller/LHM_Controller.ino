#include <LHMController.h>
#include <LHMMessage.h>
#include <LHMDataTable.h>
#include <utilities/LEDController.h>
#include <Arduino.h>

const int STATUS_REPORT_INTERVAL = 500;
const int COMMAND_IN_CHECK_INTERVAL = 1;

mavlink_system_t mavlink_system = {
    1, // System ID (1-255)
    55  // Component ID (a MAV_COMPONENT value)
};

LHMController lhmController = LHMController();
LHMMessage lhmMsg = LHMMessage(lhmController);

void setup()
{
  DEBUG_SERIAL.begin(1000000);
  // waitDebugSerial();

  lhmController.setup();
  while (!lhmMsg.initiate())
  {
    lhmController.ledRed.syncBlink(2, FLASH_TIME_SHORT_EXTREME, FLASH_TIME_SHORT_EXTREME);
    delay(1000);
  }
  if (lhmController.initiate())
  {
    lhmController.ledStat.flash(5, FLASH_TIME_SHORT_NORM, FLASH_TIME_SHORT_NORM);
  }
  else
  {
    lhmController.ledStat.flash(10, FLASH_TIME_SHORT_EXTREME, FLASH_TIME_SHORT_EXTREME);
  }
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
  lhmController.loopAllLED();
  reportStatus();
  checkCommandIn();
  trackHingeTransition();
  checkJettisonButton();
}

inline bool isTimeToReportStatus()
{
  return (int)millis() - prevStatusReportTime > STATUS_REPORT_INTERVAL;
}

inline bool isTimeToCheckCommandIn()
{
  return ((int)millis() - prevCommandInCheckTime > COMMAND_IN_CHECK_INTERVAL);
}

void reportStatus()
{
  if (!isTimeToReportStatus())
  {
    return;
  }
  prevStatusReportTime = millis();
  hgStatus = lhmController.getHingeStatus();
  hkStatus = lhmController.getHookStatus();
  bool egStatus = lhmController.isEngaged();
  lhmMsg.sendStatusMessage(hgStatus, hkStatus, (uint8_t)egStatus);
  setStatusLEDs();
  DEBUG_SERIAL.printf("【Status】Hinge=%d | Hook=%d | Engaged=%d\n", (int)hgStatus, (int)hkStatus, (int)egStatus);
}

void checkCommandIn()
{
  if (!isTimeToCheckCommandIn())
  {
    return;
  }
  prevCommandInCheckTime = millis();
  int32_t cmd = lhmMsg.readCommandIn();
  if (!validateCommand(cmd))
  {
    return;
  }
  processCommand(cmd);
}

inline bool validateCommand(int32_t cmd)
{
  if (cmd == (int32_t)CommandType::UNKNOWN)
  {
    //No message or not debug_vect type of message
    return false;
  }
  if (cmd == (int32_t)CommandType::ERROR)
  {
    //debug_vect.name indicates the message is not a CommandIn type
    lhmMsg.sendCommandFeedbackReception(cmd, false);
    return false;
  }
  return true;
}

void processCommand(int32_t cmd)
{
  bool result = false;
  switch (cmd)
  {
  case (int32_t)CommandType::JETTISON:
    lhmMsg.sendCommandFeedbackReception(cmd, true);
    result = lhmController.jettison();
    break;
  case (int32_t)CommandType::LOCK_LHM:
    lhmMsg.sendCommandFeedbackReception(cmd, true);
    result = lhmController.lockLHM();
    break;
  case (int32_t)CommandType::RESET_DYNAMIXEL_COM:
    lhmMsg.sendCommandFeedbackReception(cmd, true);
    result = lhmController.initiate();
    break;
  case (int32_t)CommandType::HINGE_POWER_OFF:
    lhmMsg.sendCommandFeedbackReception(cmd, true);
    result = lhmController.stopHingeMotor();
    break;
  case (int32_t)CommandType::HINGE_TAKE_OFF:
    lhmMsg.sendCommandFeedbackReception(cmd, true);
    result = lhmController.setTakeoffMode();
    break;
  case (int32_t)CommandType::HINGE_LANDING:
    lhmMsg.sendCommandFeedbackReception(cmd, true);
    result = hingeInTransition = lhmController.setLandingPosition();
    break;
  case (int32_t)CommandType::HINGE_SWING_REDUCTION:
    lhmMsg.sendCommandFeedbackReception(cmd, true);
    result = lhmController.setSwingReductionMode();
    break;
  case (int32_t)CommandType::HOOK_POWER_OFF:
    lhmMsg.sendCommandFeedbackReception(cmd, true);
    result = lhmController.stopHookMotor();
    break;
  case (int32_t)CommandType::HOOK_CLOSE:
    lhmMsg.sendCommandFeedbackReception(cmd, true);
    result = lhmController.closeHook();
    hookMotionStartTime = millis();
    break;
  case (int32_t)CommandType::HOOK_OPEN:
    lhmMsg.sendCommandFeedbackReception(cmd, true);
    result = lhmController.openHook();
    hookMotionStartTime = millis();
    break;
  default:
    lhmMsg.sendCommandFeedbackReception(cmd, false);
    return;
  }
  lhmMsg.sendCommandExecutionFeedback(cmd, result);
}

void trackHingeTransition()
{
  if (!hingeInTransition)
  {
    return;
  }
  MotionSequenceStatusType status = lhmController.getMotionSequenceStatus();
  DEBUG_SERIAL.printf("MotionSequenceStatus: %d\n", (int)status);

  switch (status)
  {
  case MotionSequenceStatusType::COMPLETED:
    hingeInTransition = false;
    break;
  case MotionSequenceStatusType::STAGE_COMPLETED:
    lhmController.nextMotionSequence();
    break;
  case MotionSequenceStatusType::BUSY:
    break;
  case MotionSequenceStatusType::ERROR:
  case MotionSequenceStatusType::UNKNOWN:
    hingeInTransition = false;
    break;
  }
}

void trackHookTransition()
{
  if (hookMotionStartTime > 0 && ((int)millis() - hookMotionStartTime) > hookMotionStopDelay)
  {
    HookStatus hkMotion = lhmController.getHookMotionStatus();
    if (hkMotion == HookStatus::OPENNING || hkMotion == HookStatus::CLOSING)
    {
      hkStatus = lhmController.getHookStatus();
      // DEBUG_SERIAL.printf("Status_rapid: Hook=%d\n", (int)hkStatus);
      if (hkStatus != HookStatus::OPENNING && hkStatus != HookStatus::CLOSING)
        if (lhmController.stopHookMotor())
        {
          hookMotionStartTime = 0;
        }
    }
  }
}

inline void pulseLEDSequantial()
{
  lhmController.hingeMotorPitch.flashLED(10, 200);
  lhmController.hingeMotorRoll.flashLED(10, 200);
  lhmController.hookMotor.flashLED(10, 200);
}

inline void waitDebugSerial()
{
  while (!DEBUG_SERIAL)
    ;
}

void setStatusLEDs()
{
  if (hgStatus == HingeStatus::ERROR || hgStatus == HingeStatus::OFFLINE || hgStatus == HingeStatus::UNKNOWN)
  {
    lhmController.ledBlue.set(LED_OFF_REVERSED);
  }
  else
  {
    lhmController.ledBlue.flash(-1, FLASH_TIME_LONG_EXTREME, FLASH_TIME_LONG_NORM);
  }
  if (hkStatus == HookStatus::ERROR || hkStatus == HookStatus::OFFLINE || hkStatus == HookStatus::UNKNOWN)
  {
    lhmController.ledGreen.set(LED_OFF_REVERSED);
  }
  else
  {
    lhmController.ledGreen.flash(-1, FLASH_TIME_LONG_EXTREME, FLASH_TIME_LONG_NORM);
  }
}

void checkJettisonButton()
{
  LSButtonState btnJetState = lhmController.btnJet.getState();
  if (btnJetState == LSButtonState::SHORT_PRESSED)
  {
    DEBUG_SERIAL.println("On-board button pressed, requesting locking LHM.");
    lhmController.lockLHM();
  }
  else if (btnJetState == LSButtonState::LONG_PRESSED)
  {
    DEBUG_SERIAL.println("On-board button pressed, requesting releasing LHM.");
    lhmController.jettison();
  }
}