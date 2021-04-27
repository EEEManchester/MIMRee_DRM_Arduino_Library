#include <LHMController.h>
#include <LHMMessage.h>
#include <LHMDataTable.h>
#include <utilities/LEDController.h>
#include <Arduino.h>

const int STATUS_REPORT_INTERVAL = 1000;
const int SEND_HEARTBEAT_INTERVAL = 50000000;
const int COMMAND_IN_CHECK_INTERVAL = 1;

const int MAV_COM_INITIATE_INTERVAL = 1000;
int lastMAVComInitiateTime = 0;

LHMController lhmController = LHMController();
LHMMessage lhmMsg = LHMMessage(lhmController);

mavlink_system_t mavlink_system = {
    LHM_MAV_SYS_ID,
    LHM_MAV_COMP_ID
};

void setup()
{
  DEBUG_SERIAL.begin(1000000);
  // waitDebugSerial();

  lhmController.setup();
  while (true)
  {
    if (millis() - lastMAVComInitiateTime > MAV_COM_INITIATE_INTERVAL)
    {
      bool result = lhmMsg.initiate(1);
      lhmController.ledRed.syncBlink(2, FLASH_TIME_SHORT_EXTREME, FLASH_TIME_SHORT_EXTREME);
      lastMAVComInitiateTime = millis();
      if (result)
      {
        break;
      }
    }
    
    checkJettisonButton();
  }

  if (lhmController.initiate())
  {
    lhmController.ledStat.flash(5, FLASH_TIME_SHORT_NORM, FLASH_TIME_SHORT_NORM);
  }
  else
  {
    lhmController.ledStat.flash(10, FLASH_TIME_SHORT_EXTREME, FLASH_TIME_SHORT_EXTREME);
  }

  // askProtocolVersion();

  Serial.println("【Setup】Setup complete.");
}

long prevStatusReportTime = 0;
long prevCommandInCheckTime = 0;
long prevSendHeartbeatTime = 0;
lhm_hinge_status_t hgStatus;
lhm_hook_status_t hkStatus;
long hookMotionStopDelay = 100;
long hookMotionStartTime = 0;
bool hingeInTransition = false;

void loop()
{
  lhmController.loopAllLED();
  reportStatus();
  sendHeartBeat();
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

inline bool isTimeToSendHeartbeat()
{
  return ((int)millis() - prevSendHeartbeatTime > SEND_HEARTBEAT_INTERVAL);
}

void sendHeartBeat()
{
  if (!isTimeToSendHeartbeat())
  {
    return;
  }
  prevSendHeartbeatTime = millis();
  lhmMsg.mavlink.sendHeartbeat();
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
  MAVMessage msg = lhmMsg.readMAVMessage();
  if (!validateCommand(msg))
  {
    return;
  }
  processCommand(msg);
}

inline bool validateCommand(MAVMessage &msg)
{
  if (!msg.accepted)
  {
    return false;
  }
  if (msg.msgId != MAVLINK_MSG_ID_BUTTON_CHANGE)
  {
    return false;
  }
  return true;
}

uint8_t last_cmd = LHM_CMD_ID_UNKNOWN;
uint32_t last_cmd_time = 0;
void processCommand(MAVMessage &msg)
{
  bool result = false;
  uint8_t cmd = msg.button_change.state;
  switch (cmd)
  {
  case LHM_CMD_ID_HINGE_POWER_OFF:
    lhmMsg.sendCommandFeedback(cmd, LHM_CMD_RE_SUCCESSFUL, LHM_CMD_PROG_COMMAND_ACK);
    result = lhmController.stopHingeMotor();
    break;
  case LHM_CMD_ID_HOOK_POWER_OFF:
    lhmMsg.sendCommandFeedback(cmd, LHM_CMD_RE_SUCCESSFUL, LHM_CMD_PROG_COMMAND_ACK);
    result = lhmController.stopHookMotor();
    break;
  case LHM_CMD_ID_HINGE_TAKE_OFF:
    lhmMsg.sendCommandFeedback(cmd, LHM_CMD_RE_SUCCESSFUL, LHM_CMD_PROG_COMMAND_ACK);
    result = lhmController.setTakeoffMode();
    break;
  case LHM_CMD_ID_HINGE_LANDING:
    lhmMsg.sendCommandFeedback(cmd, LHM_CMD_RE_SUCCESSFUL, LHM_CMD_PROG_COMMAND_ACK);
    if (last_cmd == cmd && millis()-last_cmd_time < 5000 && hingeInTransition)
    {
      break;
    }
    result = hingeInTransition = lhmController.setLandingPosition();
    break;
  case LHM_CMD_ID_HINGE_SWING_REDUCTION:
    lhmMsg.sendCommandFeedback(cmd, LHM_CMD_RE_SUCCESSFUL, LHM_CMD_PROG_COMMAND_ACK);
    result = lhmController.setSwingReductionMode();
    break;
  case LHM_CMD_ID_HOOK_CLOSE:
    lhmMsg.sendCommandFeedback(cmd, LHM_CMD_RE_SUCCESSFUL, LHM_CMD_PROG_COMMAND_ACK);
    result = lhmController.closeHook();
    hookMotionStartTime = millis();
    break;
  case LHM_CMD_ID_HOOK_OPEN:
    lhmMsg.sendCommandFeedback(cmd, LHM_CMD_RE_SUCCESSFUL, LHM_CMD_PROG_COMMAND_ACK);
    result = lhmController.openHook();
    hookMotionStartTime = millis();
    break;
  case LHM_CMD_ID_RESET_DYNAMIXEL_COM:
    lhmMsg.sendCommandFeedback(cmd, LHM_CMD_RE_SUCCESSFUL, LHM_CMD_PROG_COMMAND_ACK);
    result = lhmController.initiate();
    break;
  case LHM_CMD_ID_JETTISON:
    lhmMsg.sendCommandFeedback(cmd, LHM_CMD_RE_SUCCESSFUL, LHM_CMD_PROG_COMMAND_ACK);
    result = lhmController.jettison();
    break;
  case LHM_CMD_ID_LOCK_LHM:
    lhmMsg.sendCommandFeedback(cmd, LHM_CMD_RE_SUCCESSFUL, LHM_CMD_PROG_COMMAND_ACK);
    result = lhmController.lockLHM();
    break;
  default:
    lhmMsg.sendCommandFeedback(cmd, LHM_CMD_RE_FAILED, LHM_CMD_PROG_COMMAND_ACK);
    return;
  }
  last_cmd = cmd;
  last_cmd_time = millis();
  lhmMsg.sendCommandFeedback(cmd, result, LHM_CMD_PROG_MISSION_STARTED);
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
  case MS_SEQ_STATUS_COMPLETED:
    hingeInTransition = false;
    break;
  case MS_SEQ_STATUS_STAGE_COMPLETED:
    uint8_t result = lhmController.nextMotionSequence();
    // TODO handle result
    break;
  case MS_SEQ_STATUS_BUSY:
    break;
  case MS_SEQ_STATUS_ERROR:
  case MS_SEQ_STATUS_UNKNOWN:
    hingeInTransition = false;
    break;
  }
}

void trackHookTransition()
{
  if (hookMotionStartTime > 0 && ((int)millis() - hookMotionStartTime) > hookMotionStopDelay)
  {
    lhm_hook_status_t hkMotion = lhmController.getHookMotionStatus();
    if (hkMotion == LHM_HOOK_STATUS_OPENNING || hkMotion == LHM_HOOK_STATUS_CLOSING)
    {
      hkStatus = lhmController.getHookStatus();
      // DEBUG_SERIAL.printf("Status_rapid: Hook=%d\n", (int)hkStatus);
      if (hkStatus != LHM_HOOK_STATUS_OPENNING && hkStatus != LHM_HOOK_STATUS_CLOSING)
        if (lhmController.stopHookMotor())
        {
          hookMotionStartTime = 0;
        }
    }
  }
}

inline void waitDebugSerial()
{
  while (!DEBUG_SERIAL)
    ;
}

void setStatusLEDs()
{
  if (hgStatus == LHM_HINGE_STATUS_ERROR || hgStatus == LHM_HINGE_STATUS_OFFLINE || hgStatus == LHM_HINGE_STATUS_UNKNOWN)
  {
    lhmController.ledBlue.set(LED_OFF_REVERSED);
  }
  else
  {
    lhmController.ledBlue.flash(-1, FLASH_TIME_LONG_EXTREME, FLASH_TIME_LONG_NORM);
  }
  if (hkStatus == LHM_HOOK_STATUS_ERROR || hkStatus == LHM_HOOK_STATUS_OFFLINE || hkStatus == LHM_HOOK_STATUS_UNKNOWN)
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