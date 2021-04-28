#include <OlamController.h>
#include <OLAMMessage.h>
#include <OlamDataTable.h>
#include <utilities/LEDController.h>
#include <Arduino.h>

const int STATUS_REPORT_INTERVAL = 1000;
const int SEND_HEARTBEAT_INTERVAL = 1000;
const int COMMAND_IN_CHECK_INTERVAL = 1;

const int MAV_COM_INITIATE_INTERVAL = 1000;
int lastMAVComInitiateTime = 0;

OLAMController olamController = OLAMController();
OLAMMessage olamMsg = OLAMMessage(olamController);

mavlink_system_t mavlink_system = {
    OLAM_MAV_SYS_ID,
    OLAM_MAV_COMP_ID
};

void setup()
{
  DEBUG_SERIAL.begin(1000000);
  waitDebugSerial();
  olamController.setup();
  while (true)
  {
    if (millis() - lastMAVComInitiateTime > MAV_COM_INITIATE_INTERVAL)
    {
      bool result = olamMsg.initiate(1);
      olamController.ledRed.syncBlink(2, FLASH_TIME_SHORT_EXTREME, FLASH_TIME_SHORT_EXTREME);
      lastMAVComInitiateTime = millis();
      if (result)
      {
        break;
      }
    }
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

long prevStatusReportTime = 0;
long prevCommandInCheckTime = 0;
long prevSendHeartbeatTime = 0;
uint8_t tcStatus;
uint8_t commandSeq = 0;
uint8_t commandExeResult = 0;

void loop()
{
  olamController.loopAllLED();
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