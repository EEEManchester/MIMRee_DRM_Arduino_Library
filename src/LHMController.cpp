#include "LHMController.h"

LHMController::LHMController()
    : dxl(Dynamixel2Arduino(DXL_SERIAL, PIN_DXL_DIR)),
      lhmMessage(LHMMessage()),
      hookMotor(DXLMotor(dxl, MOTOR_ID_HOOK, DEBUG_SERIAL)),
      hingeMotorPitch(DXLMotor(dxl, MOTOR_ID_HINGE_PITCH, DEBUG_SERIAL)),
      hingeMotorRoll(DXLMotor(dxl, MOTOR_ID_HINGE_ROLL, DEBUG_SERIAL))
{    
    motors[0] = &hookMotor;
    if (MOTOR_ID_HINGE_PITCH == 2)
    {
        motors[1] = &hingeMotorPitch;
        motors[2] = &hingeMotorRoll;
    }
    else
    {
        motors[2] = &hingeMotorPitch;
        motors[1] = &hingeMotorRoll;
    }
}

void LHMController::initiate()
{
    LHMController::dxl.begin(DXL_BAUD_RATE);
    LHMController::dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    LHMController::hingeMotorPitch.reboot();
    LHMController::hingeMotorRoll.reboot();
    LHMController::hookMotor.reboot();
    LHMController::hookMotor.setOperatingMode(OP_VELOCITY);
    stopHookMotor();
    stopHingeMotor();

    pinMode(PIN_LIMIT_SWITCH_CLOSED_BOT, INPUT_PULLDOWN);
    pinMode(PIN_LIMIT_SWITCH_CLOSED_TOP, INPUT_PULLDOWN);
    pinMode(PIN_LIMIT_SWITCH_OPEN_BOT, INPUT_PULLDOWN);
    pinMode(PIN_LIMIT_SWITCH_OPEN_TOP, INPUT_PULLDOWN);

    int servoMin, servoMax;
    if (JETTISON_SERVO_VALUE_CLOSE < JETTISON_SERVO_VALUE_OPEN)
    {
        servoMin = JETTISON_SERVO_VALUE_CLOSE;
        servoMax = JETTISON_SERVO_VALUE_OPEN;
    }
    else
    {
        servoMin = JETTISON_SERVO_VALUE_OPEN;
        servoMax = JETTISON_SERVO_VALUE_CLOSE;
    }
    jettisonServo.attach(PIN_JETTISON_SERVO_PWM, servoMin, servoMax);
}

HingeStatus LHMController::getHingeStatus()
{
    Serial.printf("LHMController::getHingeStatus: hingeMotorPitch -> record: OP[%d] | Pos[%f] | Vel[%f] | VelProf[%d] | AccProf[%d]\n", 
    (int)hingeMotorPitch.getLastSetOperatingMode(),
    hingeMotorPitch.getLastSetGoalPosition(),
    hingeMotorPitch.getLastSetGoalVelocity(),
    hingeMotorPitch.getLastSetVelocityProfile(),
    hingeMotorPitch.getLastSetAccelerationProfile());
    Serial.printf("LHMController::getHingeStatus: hingeMotorRoll -> record: OP[%d] | Pos[%f] | Vel[%f] | VelProf[%d] | AccProf[%d]\n", 
    (int)hingeMotorRoll.getLastSetOperatingMode(),
    hingeMotorRoll.getLastSetGoalPosition(),
    hingeMotorRoll.getLastSetGoalVelocity(),
    hingeMotorRoll.getLastSetVelocityProfile(),
    hingeMotorRoll.getLastSetAccelerationProfile());
    if (!LHMController::hingeMotorPitch.isOnline() || !LHMController::hingeMotorRoll.isOnline())
    {
        return HingeStatus::OFFLINE;
    }
    
    bool torquePitch = LHMController::hingeMotorPitch.isTorqueOn();
    bool torqueRoll = LHMController::hingeMotorRoll.isTorqueOn();
    if (!torquePitch)
    {
        if (!torqueRoll)
            return HingeStatus::TAKEOFF_MODE;
        else
            return HingeStatus::ERROR;
    }
    OperatingMode opPitch = LHMController::hingeMotorPitch.getLastSetOperatingMode();
    OperatingMode opRoll = LHMController::hingeMotorRoll.getLastSetOperatingMode();
    if (opPitch == OP_CURRENT)
    {
        if (opRoll == OP_CURRENT)
            return HingeStatus::SWING_REDUCTION;
        else
            return HingeStatus::ERROR;
    }
    if (opRoll == OP_POSITION)
    {
        if (currentMotionSequence.sequenceType() == MotionSequenceType::UNKNOWN)
            return HingeStatus::ERROR;
        if (currentMotionSequence.sequenceType() == MotionSequenceType::LANDING)
        {
            MotionSequenceStatusType status = currentMotionSequence.status();
            if (status == MotionSequenceStatusType::COMPLETED)
            {
                return HingeStatus::LANDING_POSITION_READY;
            }
            else if (status == MotionSequenceStatusType::BUSY || status == MotionSequenceStatusType::STAGE_COMPLETED)
            {
                return HingeStatus::LANDING_POSITION_IN_TRANSITION;
            }
            else if (status == MotionSequenceStatusType::ERROR)
            {
                return HingeStatus::ERROR;
            }
            else
            {
                return HingeStatus::UNKNOWN;
            }
        }
        else
            return HingeStatus::ERROR;
    }
}

HookStatus LHMController::getHookStatus()
{
    LimitSwitchStatus top_ls = getTopLimitSwitchStatus();
    LimitSwitchStatus bot_ls = getBotLimitSwitchStatus();
    DEBUG_SERIAL.printf("LHMController::getHookStatus::top=[%d], bot=[%d]\n", top_ls, bot_ls);
    if (top_ls == LimitSwitchStatus::ERROR || bot_ls == LimitSwitchStatus::ERROR)
        return HookStatus::ERROR;
    if (top_ls == LimitSwitchStatus::CLOSED && bot_ls == LimitSwitchStatus::OPEN)
    {
        return HookStatus::FULLY_OPEN;
    }
    if (top_ls == LimitSwitchStatus::OPEN && bot_ls == LimitSwitchStatus::CLOSED)
    {
        return HookStatus::FULLY_CLOSED;
    }
    if (top_ls == LimitSwitchStatus::CLOSED && bot_ls == LimitSwitchStatus::CLOSED)
    {
        return HookStatus::ERROR;
    }
    if (top_ls == LimitSwitchStatus::OFFLINE || bot_ls == LimitSwitchStatus::OFFLINE || !hookMotor.isOnline())
    {
        return HookStatus::OFFLINE;
    }
    if (LHMController::hookMotor.isTorqueOn())
    {
        return LHMController::hookMotionStatus;
    }
    return HookStatus::LOOSE;
}

bool LHMController::isEngaged()
{
    return getPESensorStatus() == OnOffStatus::ON;
}

bool LHMController::setSwingReductionMode()
{
    bool result = LHMController::hingeMotorPitch.setOperatingMode(OP_CURRENT);
    result = result && LHMController::hingeMotorPitch.setTorqueOn();
    result = result && LHMController::hingeMotorPitch.setGoalCurrent(0);
    result = result && LHMController::hingeMotorRoll.setOperatingMode(OP_CURRENT);
    result = result && LHMController::hingeMotorRoll.setTorqueOn();
    result = result && LHMController::hingeMotorRoll.setGoalCurrent(0);
    DEBUG_SERIAL.printf("LHMController::setSwingReductionMode: %s\n", result ? "Successful" : "Failed");
    LHMController::lhmMessage.sendCommandFeedback(CommandType::HINGE_SWING_REDUCTION, result);
    return result;
}

bool LHMController::setLandingPosition()
{
    currentMotionSequence = MotionSequence(MotionSequenceType::LANDING, motors, MOTION_SEQ_LANDING);
    return currentMotionSequence.next() == 1;
}

bool LHMController::isAtLandingPosition()
{
    return LHMController::hingeMotorPitch.isAtPosition(HINGE_X_VAL_LANDING_POISITION);
}

MotionSequenceStatusType LHMController::getMotionSequenceStatus()
{
    if (currentMotionSequence.sequenceType() == MotionSequenceType::UNKNOWN)
    {
        return MotionSequenceStatusType::UNKNOWN;
    }
    return currentMotionSequence.status();
}

int8_t LHMController::nextMotionSequence()
{
    if (currentMotionSequence.sequenceType() == MotionSequenceType::UNKNOWN)
    {
        return -1;
    }
    return currentMotionSequence.next();
}

bool LHMController::setTakeoffMode()
{
    bool result = LHMController::hingeMotorPitch.setTorqueOff();
    result = result && LHMController::hingeMotorRoll.setTorqueOff();
    DEBUG_SERIAL.printf("LHMController::setTakeoffMode: %s\n", result ? "Successful" : "Failed");
    LHMController::lhmMessage.sendCommandFeedback(CommandType::HINGE_TAKE_OFF, result);
    return result;
}

bool LHMController::stopHingeMotor()
{
    bool result = LHMController::hingeMotorPitch.setTorqueOff();
    result = result && LHMController::hingeMotorRoll.setTorqueOff();
    DEBUG_SERIAL.printf("LHMController::stopHingeMotor: %s\n", result ? "Successful" : "Failed");
    LHMController::lhmMessage.sendCommandFeedback(CommandType::HINGE_POWER_OFF, result);
    return result;
}

bool LHMController::openHook()
{
    HookStatus hStatus = getHookStatus();
    if (hStatus == HookStatus::FULLY_OPEN)
    {
        return true;
    }
    if (hStatus == HookStatus::OFFLINE || hStatus == HookStatus::ERROR)
    {
        return false;
    }
    //TODO make this none blocking
    bool result = LHMController::hookMotor.setTorqueOn();
    result = result && LHMController::hookMotor.setGoalVelocity(VELOCITY_HOOK_MOTOR_OPEN);
    if (result)
    {
        LHMController::hookMotionStatus = HookStatus::OPENNING;
    }
    return result;
}

bool LHMController::closeHook()
{
    HookStatus hStatus = getHookStatus();
    if (hStatus == HookStatus::FULLY_CLOSED)
    {
        return true;
    }
    if (hStatus == HookStatus::OFFLINE || hStatus == HookStatus::ERROR)
    {
        return false;
    }

    //TODO make this none blocking
    bool result = LHMController::hookMotor.setTorqueOn();
    result = result && LHMController::hookMotor.setGoalVelocity(VELOCITY_HOOK_MOTOR_CLOSE);
    if (result)
    {
        LHMController::hookMotionStatus = HookStatus::CLOSING;
    }
    return result;
}

bool LHMController::stopHookMotor()
{
    bool result = LHMController::hookMotor.setGoalVelocity(0);
    delay(100);
    result = LHMController::hookMotor.setTorqueOff() && result;
    return result;
}

bool LHMController::jettison()
{
    for (int i = 0; i < 10; i++)
    {
        jettisonServo.write(JETTISON_SERVO_VALUE_OPEN);
        delay(100);
    }
    return true;
}

bool LHMController::lockLHM()
{
    for (int i = 0; i < 10; i++)
    {
        jettisonServo.write(JETTISON_SERVO_VALUE_CLOSE);
        delay(100);
    }
    return true;
}

LimitSwitchStatus LHMController::getBotLimitSwitchStatus()
{
    return getLimitSwitchStatus(PIN_LIMIT_SWITCH_CLOSED_BOT, PIN_LIMIT_SWITCH_OPEN_BOT);
}

LimitSwitchStatus LHMController::getTopLimitSwitchStatus()
{
    return getLimitSwitchStatus(PIN_LIMIT_SWITCH_CLOSED_TOP, PIN_LIMIT_SWITCH_OPEN_TOP);
}

LimitSwitchStatus LHMController::getLimitSwitchStatus(int closed_pin, int open_pin)
{
    bool closed_pin_on = digitalReadExt(closed_pin) == HIGH;
    bool open_pin_on = digitalReadExt(open_pin) == HIGH;
    LimitSwitchStatus status;
    if (closed_pin_on && !open_pin_on)
        status = LimitSwitchStatus::CLOSED;
    else if (!closed_pin_on && open_pin_on)
        status = LimitSwitchStatus::OPEN;
    else if (closed_pin_on && open_pin_on)
        status = LimitSwitchStatus::ERROR;
    else
        status = LimitSwitchStatus::OFFLINE;
    DEBUG_SERIAL.printf("LHMController::getLimitSwitchStatus::pin[%d & %d] -> %d\n", closed_pin, open_pin, (int)status);
    return status;
}

OnOffStatus LHMController::getPESensorStatus()
{
    DEBUG_SERIAL.println("LHMController::getPESensorStatus");
    if (digitalReadExt(PIN_PE_SENSOR) == HIGH)
    {
        return OnOffStatus::ON;
    }
    return OnOffStatus::OFF;
}

int LHMController::digitalReadExt(int pin)
{
    int state = digitalRead(pin);
    DEBUG_SERIAL.printf("LHMController::digitalReadExt::pin[%d]=%d\n", pin, state);
    return state;
}