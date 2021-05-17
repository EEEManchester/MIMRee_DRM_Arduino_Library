#include <OlamController.h>
#include <OLAMMessage.h>
#include <OlamDataTable.h>
#include <utilities/LEDController.h>
#include <Arduino.h>

const uint32_t STATUS_REPORT_INTERVAL = 1000;
const uint32_t SEND_HEARTBEAT_INTERVAL = 1000;
const uint32_t COMMAND_IN_CHECK_INTERVAL = 1;

const uint32_t MAV_COM_INITIATE_INTERVAL = 1000;
uint32_t lastMAVComInitiateTime = 0;

#define LHM_VERSION "V1.1-20210429-1736"

OLAMController olamController = OLAMController();
OLAMMessage olamMsg = OLAMMessage(olamController);

mavlink_system_t mavlink_system = {
    OLAM_MAV_SYS_ID,
    OLAM_MAV_COMP_ID
};

void setup()
{
  DEBUG_SERIAL.begin(1000000);
  DEBUG_SERIAL.printf("Version: %s\n", LHM_VERSION);
  // waitDebugSerial();
  olamController.setup();
  while (!olamMsg.initiate())
  {
    Serial.println("Connection failed. Trying again.");
    checkOnBoardButton();
    delay(1000);
  }

  if (olamController.initiate())
  {
    olamController.ledStat.flash(5, FLASH_TIME_SHORT_NORM, FLASH_TIME_SHORT_NORM);
  }
  else
  {
    olamController.ledStat.flash(10, FLASH_TIME_SHORT_EXTREME, FLASH_TIME_SHORT_EXTREME);
  }

  // askProtocolVersion();

  Serial.println("【Setup】Setup complete.");
}

uint32_t prevStatusReportTime = 0;
uint32_t prevCommandInCheckTime = 0;
uint32_t prevSendHeartbeatTime = 0;
uint8_t tcStatus;
uint8_t commandSeq = 0;
uint8_t commandExeResult = 0;

void loop()
{
  olamController.loopAllLED();
  checkOnBoardButton();
  reportStatus();
  sendHeartBeat();
  checkCommandIn();
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
  olamMsg.mavlink.sendHeartbeat();
}

void reportStatus()
{
  if (!isTimeToReportStatus())
  {
    return;
  }
  prevStatusReportTime = millis();
  tcStatus = olamController.ltController.status();
  olamMsg.sendStatusMessage(tcStatus, commandSeq, commandExeResult);
  DEBUG_SERIAL.printf("【Status】TCStatus=%d, commandSeq=%d, CMDExecusion=%d\n", tcStatus, commandSeq, commandExeResult);
}

void checkCommandIn()
{
  if (!isTimeToCheckCommandIn())
  {
    return;
  }
  prevCommandInCheckTime = millis();
  MAVMessage msg = olamMsg.readMAVMessage();
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

void processCommand(MAVMessage &msg)
{
  bool result = false;
  uint8_t cmd = msg.button_change.state;
  uint32_t sTime = millis();
  commandExeResult = 0;
  commandSeq += 1;
  switch (cmd)
  {
  case OLAM_TC_CMD_ID_POWER_OFF:
    olamMsg.sendCommandFeedback(cmd, OLAM_CMD_RE_SUCCESSFUL, OLAM_CMD_PROG_COMMAND_ACK);
    result = olamController.ltController.powerOff();
    break;
  case OLAM_TC_CMD_ID_PREPARE_FOR_ENGAGEMENT:
    olamMsg.sendCommandFeedback(cmd, OLAM_CMD_RE_SUCCESSFUL, OLAM_CMD_PROG_COMMAND_ACK);
    result = olamController.ltController.prepareEngagement();
    break;
  case OLAM_TC_CMD_ID_DETENSION:
    olamMsg.sendCommandFeedback(cmd, OLAM_CMD_RE_SUCCESSFUL, OLAM_CMD_PROG_COMMAND_ACK);
    result = olamController.ltController.detension();
    break;
  case OLAM_TC_CMD_ID_HOME:
    olamMsg.sendCommandFeedback(cmd, OLAM_CMD_RE_SUCCESSFUL, OLAM_CMD_PROG_COMMAND_ACK);
    result = olamController.ltController.goToHomeAndHold();
    break;
  case OLAM_TC_CMD_ID_HOLD_POSITION:
    olamMsg.sendCommandFeedback(cmd, OLAM_CMD_RE_SUCCESSFUL, OLAM_CMD_PROG_COMMAND_ACK);
    result = olamController.ltController.holdPosition();
    break;
  default:
    olamMsg.sendCommandFeedback(cmd, OLAM_CMD_RE_FAILED, OLAM_CMD_PROG_COMMAND_ACK);
    return;
  }
  // DEBUG_SERIAL.printf("【CommandFeedback】 CMD_id = %d, result = %d\n", cmd, (int)result);
  // olamMsg.sendCommandFeedback(cmd, result, OLAM_CMD_PROG_MISSION_FINISHED);
  commandExeResult = result ? 2: 1;
}

inline void waitDebugSerial()
{
  while (!DEBUG_SERIAL)
    ;
}

void checkOnBoardButton()
{
  LSButtonState btnHomeState = olamController.homeButton.getState();
  if (btnHomeState == LSButtonState::LONG_PRESSED)
  {
    DEBUG_SERIAL.println("On-board [HOME] button long pressed, requesting HOME procedures.");
    olamController.ltController.goToHomeAndHold();
  }
  else if (btnHomeState == LSButtonState::SHORT_PRESSED)
  {
    DEBUG_SERIAL.println("On-board [HOME] button short pressed, requesting POWER OFF.");
    olamController.ltController.powerOff();
  }  
  LSButtonState btnEngagePrepState = olamController.engagementButton.getState();
  if (btnEngagePrepState == LSButtonState::LONG_PRESSED)
  {
    DEBUG_SERIAL.println("On-board [ENG] button long pressed, requesting ENGAGEMENT-PREP procedures.");
    olamController.ltController.prepareEngagement();
  }
  else if (btnEngagePrepState == LSButtonState::SHORT_PRESSED)
  {
    DEBUG_SERIAL.println("On-board [ENG] button short pressed, requesting POWER OFF.");
    olamController.ltController.powerOff();
  }

}