#include "LHMController.h"

LHMController::LHMController()
    : dxl(Dynamixel2Arduino(DXL_SERIAL, PIN_DXL_DIR)),
      hookMotor(DXLMotor(dxl, MOTOR_ID_HOOK)),
      hingeMotorPitch(DXLMotor(dxl, MOTOR_ID_HINGE_PITCH)),
      hingeMotorRoll(DXLMotor(dxl, MOTOR_ID_HINGE_ROLL)),
      btnJet(PIN_BUTTON_JETTISON)
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

void LHMController::setup()
{
    dxl.begin(DXL_BAUD_RATE);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    pinMode(PIN_LIMIT_SWITCH_CLOSED_BOT, INPUT_PULLDOWN);
    pinMode(PIN_LIMIT_SWITCH_CLOSED_TOP, INPUT_PULLDOWN);
    pinMode(PIN_LIMIT_SWITCH_OPEN_BOT, INPUT_PULLDOWN);
    pinMode(PIN_LIMIT_SWITCH_OPEN_TOP, INPUT_PULLDOWN);

    setupOnBoardDevices();
    // btnJet.setup();

    uint8_t servoMin, servoMax;
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

bool LHMController::initiate()
{
    bool result = true;
    result = result & hingeMotorPitch.reboot();
    result = result & hingeMotorRoll.reboot();
    result = result & hookMotor.reboot();
    result = result & hookMotor.setOperatingMode(OP_VELOCITY);
    result = result & stopHookMotor();
    result = result & stopHingeMotor();
    DEBUG_SERIAL.println(result ? "LHMController::initiateDXL: DXL servos initiated." : "LHMController::initiateDXL: Fail to initiated DXL servos.");
    return result;
}

HingeStatus LHMController::getHingeStatus()
{
    LHM_DEBUG_PRINTF("LHMController::getHingeStatus: hingeMotorPitch -> record: OP[%d] | Pos[%f] | Vel[%f] | VelProf[%d] | AccProf[%d]\n",
                     (int)hingeMotorPitch.getLastSetOperatingMode(),
                     hingeMotorPitch.getLastSetGoalPosition(),
                     hingeMotorPitch.getLastSetGoalVelocity(),
                     hingeMotorPitch.getLastSetVelocityProfile(),
                     hingeMotorPitch.getLastSetAccelerationProfile());
    LHM_DEBUG_PRINTF("LHMController::getHingeStatus: hingeMotorRoll -> record: OP[%d] | Pos[%f] | Vel[%f] | VelProf[%d] | AccProf[%d]\n",
                     (int)hingeMotorRoll.getLastSetOperatingMode(),
                     hingeMotorRoll.getLastSetGoalPosition(),
                     hingeMotorRoll.getLastSetGoalVelocity(),
                     hingeMotorRoll.getLastSetVelocityProfile(),
                     hingeMotorRoll.getLastSetAccelerationProfile());

    if (!hingeMotorPitch.isOnline() || !hingeMotorRoll.isOnline())
    {
        return HingeStatus::OFFLINE;
    }

    bool torquePitch = hingeMotorPitch.isTorqueOn();
    bool torqueRoll = hingeMotorRoll.isTorqueOn();
    if (!torquePitch)
    {
        if (!torqueRoll)
            return HingeStatus::TAKEOFF_MODE;
        else
            return HingeStatus::ERROR;
    }
    OperatingMode opPitch = hingeMotorPitch.getLastSetOperatingMode();
    OperatingMode opRoll = hingeMotorRoll.getLastSetOperatingMode();
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
    LHM_DEBUG_PRINTF("LHMController::getHookStatus::top=[%d], bot=[%d]\n", top_ls, bot_ls);

    if (top_ls == LimitSwitchStatus::ERROR || bot_ls == LimitSwitchStatus::ERROR)
        return HookStatus::ERROR;
    if (top_ls == LimitSwitchStatus::CLOSED && bot_ls == LimitSwitchStatus::OPEN)
    {
        if (hookMotor.isTorqueOn() && hookMotionStatus == HookStatus::CLOSING)
            return HookStatus::CLOSING;
        return HookStatus::FULLY_OPEN;
    }
    if (top_ls == LimitSwitchStatus::OPEN && bot_ls == LimitSwitchStatus::CLOSED)
    {
        if (hookMotor.isTorqueOn() && hookMotionStatus == HookStatus::OPENNING)
            return HookStatus::OPENNING;
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
    if (hookMotor.isTorqueOn())
    {
        return hookMotionStatus;
    }
    return HookStatus::LOOSE;
}

bool LHMController::setSwingReductionMode()
{
    bool result = hingeMotorPitch.setOperatingMode(OP_CURRENT);
    result = result && hingeMotorPitch.setTorqueOn();
    result = result && hingeMotorPitch.setGoalCurrent(0);
    result = result && hingeMotorRoll.setOperatingMode(OP_CURRENT);
    result = result && hingeMotorRoll.setTorqueOn();
    result = result && hingeMotorRoll.setGoalCurrent(0);
    LHM_DEBUG_PRINTF("LHMController::setSwingReductionMode: %s\n", result ? "Successful" : "Failed");
    return result;
}

bool LHMController::setLandingPosition()
{
    currentMotionSequence = MotionSequence(MotionSequenceType::LANDING, motors, MOTION_SEQ_LANDING);
    return currentMotionSequence.next() == 1;
}

bool LHMController::isAtLandingPosition()
{
    if (currentMotionSequence.sequenceType() == MotionSequenceType::LANDING)
    {
        return currentMotionSequence.status() == MotionSequenceStatusType::COMPLETED;
    }
    else
    {
        return false;
    }
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
    bool result = hingeMotorPitch.setTorqueOff();
    result = result && hingeMotorRoll.setTorqueOff();
    LHM_DEBUG_PRINTF("LHMController::setTakeoffMode: %s\n", result ? "Successful" : "Failed");
    return result;
}

bool LHMController::stopHingeMotor()
{
    bool result = hingeMotorPitch.setTorqueOff();
    result = result && hingeMotorRoll.setTorqueOff();
    LHM_DEBUG_PRINTF("LHMController::stopHingeMotor: %s\n", result ? "Successful" : "Failed");
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
    bool result = hookMotor.setTorqueOn();
    result = result && hookMotor.setGoalVelocity(VELOCITY_HOOK_MOTOR_OPEN);
    if (result)
    {
        hookMotionStatus = HookStatus::OPENNING;
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

    bool result = hookMotor.setTorqueOn();
    result = result && hookMotor.setGoalVelocity(VELOCITY_HOOK_MOTOR_CLOSE);
    if (result)
    {
        hookMotionStatus = HookStatus::CLOSING;
    }
    return result;
}

bool LHMController::stopHookMotor()
{
    bool result = hookMotor.setGoalVelocity(0);
    delay(100);
    result = hookMotor.setTorqueOff() && result;
    if (result)
    {
        hookMotionStatus = HookStatus::UNKNOWN;
    }
    return result;
}

bool LHMController::jettison()
{
    for (uint8_t i = 0; i < 10; i++)
    {
        jettisonServo.write(JETTISON_SERVO_VALUE_OPEN);
        delay(100);
    }
    return true;
}

bool LHMController::lockLHM()
{
    for (uint8_t i = 0; i < 10; i++)
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

LimitSwitchStatus LHMController::getLimitSwitchStatus(uint8_t closed_pin, uint8_t open_pin, bool offlineRetry)
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
    else if (offlineRetry)
    {
        delay(10);
        status = getLimitSwitchStatus(closed_pin, open_pin, false);
    }
    else
    {
        status = LimitSwitchStatus::OFFLINE;
    }
    LHM_DEBUG_PRINTF("LHMController::getLimitSwitchStatus::pin[%d & %d] -> %d\n", closed_pin, open_pin, (int)status);
    return status;
}

OnOff LHMController::getPESensorStatus()
{
    if (digitalReadExt(PIN_PE_SENSOR) == HIGH)
    {
        return OnOff::ON;
    }
    return OnOff::OFF;
}