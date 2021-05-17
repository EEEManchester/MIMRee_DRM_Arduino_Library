#include <LHMController.h>
#include <LHMMessage.h>
#include <LHMDataTable.h>
#include <utilities/LEDController.h>
#include <Arduino.h>
#include <DataWriter.h>
#include <cppQueue.h>

#define LHM_VERSION "V1.0-20210512-2257"
#define LHM_LOG_DIR "LHM"
#define CPP_QUEUE_IMPLEMENTATION FIFO

const uint32_t STATUS_REPORT_INTERVAL = 1000;
const uint32_t SEND_HEARTBEAT_INTERVAL = 1000;
const uint32_t COMMAND_IN_CHECK_INTERVAL = 1;
const uint32_t DATA_LOG_INTERVAL = 50;
const uint32_t COMMAND_ACK_DELAY_TIMER = 1000;

mavlink_system_t mavlink_system = {
    LHM_MAV_SYS_ID,
    LHM_MAV_COMP_ID};
typedef struct Datastore
{
  uint32_t timestamp;
  float servo_roll_current;
  float servo_roll_position;
  float servo_roll_velocity;
  float servo_pitch_current;
  float servo_pitch_position;
  float servo_pitch_velocity;
  uint8_t hingeStatus;
  uint8_t hookStatus;
  bool engaged;
  uint8_t lastCommand;
  uint32_t lastCommandTime;
} lhm_datastore_t;

typedef struct CommandACKData
{
  uint32_t dueMillis;
  uint8_t cmd;
  bool result;
  uint8_t prog;
} command_ack_data_t;

DataWriter logWriter(LHM_LOG_DIR);
uint32_t timeSyncAdjustment = 0;
lhm_datastore_t datastore;

LHMController lhmController = LHMController();
LHMMessage lhmMsg = LHMMessage(lhmController);

cppQueue cmdACKQueue(sizeof(CommandACKData), 256, CPP_QUEUE_IMPLEMENTATION);

uint32_t prevStatusReportTime = 0;
uint32_t prevCommandInCheckTime = 0;
uint32_t prevSendHeartbeatTime = 0;
uint32_t prevDataLogTime = 0;
lhm_hinge_status_t hgStatus;
lhm_hook_status_t hkStatus;
uint32_t hookMotionStopDelay = 100;
uint32_t hookMotionStartTime = 0;
bool hingeInTransition = false;

void setup()
{
  DEBUG_SERIAL.begin(1000000);
  DEBUG_SERIAL.printf("Version: %s\n", LHM_VERSION);
  // waitDebugSerial();

  lhmController.setup();

  while (!lhmMsg.initiate())
  {
    Serial.println("Connection failed. Retrying after 1s.");
    delay(1000);
  }

  logWriter.setup();
  logWriter.enabled = true;
  Serial.println("【Setup】Complete.");
}

void loop()
{
  lhmController.loopAllLED();
  reportStatus();
  sendHeartBeat();
  checkCommandIn();
  trackHingeTransition();
  trackHookTransition();
  checkJettisonButton();
  checkSDLoggerButton();
  logLHMData();
  checkCommandACKQueue();
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
  DEBUG_SERIAL.printf("【Status】Hinge=%d | Hook=%d | Engaged=%d\n", (unsigned)hgStatus, (unsigned)hkStatus, (unsigned)egStatus);
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

inline bool validateCommand(const MAVMessage &msg)
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
void processCommand(const MAVMessage &msg)
{
  bool result = false;
  uint8_t cmd = msg.button_change.state;
  uint32_t sTime = millis();
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
    if (last_cmd == cmd && millis() - last_cmd_time < 5000 && hingeInTransition)
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
  case LHM_CMD_ID_RESET_ALL_SERVOS:
    lhmMsg.sendCommandFeedback(cmd, LHM_CMD_RE_SUCCESSFUL, LHM_CMD_PROG_COMMAND_ACK);
    result = lhmController.resetAllServos();
    break;
  case LHM_CMD_ID_RESET_HINGE_SERVO_ERROR:
    lhmMsg.sendCommandFeedback(cmd, LHM_CMD_RE_SUCCESSFUL, LHM_CMD_PROG_COMMAND_ACK);
    result = lhmController.resetHingeServoError();
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
  if (result)
  { //Force saving data log once after receiving a new command.
    logWriter.flush();
  }
  last_cmd = cmd;
  last_cmd_time = millis();
  if (millis() - sTime < COMMAND_ACK_DELAY_TIMER)
  {
    command_ack_data_t data;
    data.dueMillis = sTime + COMMAND_ACK_DELAY_TIMER;
    data.cmd = cmd;
    data.result = result;
    data.prog = LHM_CMD_PROG_MISSION_STARTED;
    cmdACKQueue.push(&data);
  }
  else
  {
    lhmMsg.sendCommandFeedback(cmd, result, LHM_CMD_PROG_MISSION_STARTED);
    DEBUG_SERIAL.printf("【CommandFeedback】 CMD_id = %d, result = %d\n", cmd, (unsigned)result);
  }
}

void addToCommandACKQueue(const command_ack_data_t &data)
{
  if (cmdACKQueue.isFull())
  {
    command_ack_data_t dataToSend;
    cmdACKQueue.pop(&dataToSend);
    sendCommandACKData(dataToSend);
  }
  cmdACKQueue.push(&data);
}

void checkCommandACKQueue()
{
  if (cmdACKQueue.isEmpty())
  {
    return;
  }
  command_ack_data_t data;
  while (cmdACKQueue.peek(&data))
  {
    if (data.dueMillis > millis())
    {
      break;
    }
    sendCommandACKData(data);
    cmdACKQueue.drop();
  }
}

void sendCommandACKData(const command_ack_data_t &data)
{
  lhmMsg.sendCommandFeedback(data.cmd, data.result, data.prog);
  DEBUG_SERIAL.printf("【CommandFeedback】 CMD_id = %d, result = %d\n", data.cmd, (unsigned)(data.result));
}

void trackHingeTransition()
{
  if (!hingeInTransition)
  {
    return;
  }
  MotionSequenceStatusType status = lhmController.getMotionSequenceStatus();
  // DEBUG_SERIAL.printf("MotionSequenceStatus: %d\n", (int)status);
  uint8_t result = 100;
  switch (status)
  {
  case MS_SEQ_STATUS_COMPLETED:
    hingeInTransition = false;
    break;
  case MS_SEQ_STATUS_STAGE_COMPLETED:
    result = lhmController.nextMotionSequence();
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

void checkSDLoggerButton()
{
  LSButtonState btnSDState = lhmController.btnSD.getState();
  if (btnSDState == LSButtonState::LONG_PRESSED)
  {
    logWriter.setup(LHM_LOG_FNAME);
  }
}

inline bool isTimeToLogData()
{
  return (int)millis() - prevDataLogTime > DATA_LOG_INTERVAL;
}

void logLHMData()
{
  if 
  if (!logWriter.canWrite() || !isTimeToLogData())
  {
    return;
  }
  prevDataLogTime = millis();
  packDataToLog();
  logWriter.logData((uint8_t *)&datastore, sizeof(datastore));
}

void packDataToLog()
{
  datastore.timestamp = millis() + timeSyncAdjustment;
  hgStatus = lhmController.getHingeStatus();
  hkStatus = lhmController.getHookStatus();
  datastore.servo_roll_current = lhmController.hingeMotorRoll.getCurrentCurrent();
  datastore.servo_roll_position = lhmController.hingeMotorRoll.getCurrentPosition();
  datastore.servo_roll_velocity = lhmController.hingeMotorRoll.getCurrentVelocity();
  datastore.servo_pitch_current = lhmController.hingeMotorPitch.getCurrentCurrent();
  datastore.servo_pitch_position = lhmController.hingeMotorPitch.getCurrentPosition();
  datastore.servo_pitch_velocity = lhmController.hingeMotorPitch.getCurrentVelocity();
  datastore.hingeStatus = hgStatus;
  datastore.hookStatus = hkStatus;
  datastore.engaged = lhmController.isEngaged();
  datastore.lastCommand = last_cmd;
  datastore.lastCommandTime = last_cmd_time;
  // Serial.printf("[DEBUG] Packing data takes %.3f milliseconds.\n", millis()-datastore.timestamp);
  // Serial.printf("PCur=%.3f,PPos=%.3f,PVel=%.3f,RCur=%.3f,RPos=%.3f,RVel=%.3f\n", datastore.servo_pitch_current, datastore.servo_pitch_position, datastore.servo_pitch_velocity, datastore.servo_roll_current, datastore.servo_roll_position, datastore.servo_roll_velocity);
}