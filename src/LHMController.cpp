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
    pinMode(PIN_PE_SENSOR, INPUT_PULLDOWN);

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
    DEBUG_SERIAL.println(result ? "LHMController::initiate: DXL servos initiated." : "LHMController::initiateDXL: Fail to initiated DXL servos.");
    return result;
}

lhm_hinge_status_t LHMController::getHingeStatus()
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
        return LHM_HINGE_STATUS_OFFLINE;
    }

    bool torquePitch = hingeMotorPitch.isTorqueOn();
    bool torqueRoll = hingeMotorRoll.isTorqueOn();
    if (!torquePitch)
    {
        if (!torqueRoll)
            return LHM_HINGE_STATUS_TAKEOFF_MODE;
        else
            return LHM_HINGE_STATUS_ERROR;
    }
    OperatingMode opPitch = hingeMotorPitch.getLastSetOperatingMode();
    OperatingMode opRoll = hingeMotorRoll.getLastSetOperatingMode();
    if (opPitch == OP_CURRENT)
    {
        if (opRoll == OP_CURRENT)
            return LHM_HINGE_STATUS_SWING_REDUCTION;
        else
            return LHM_HINGE_STATUS_ERROR;
    }
    if (opRoll == OP_POSITION)
    {
        if (currentMotionSequence.sequenceType() == MS_SEQ_TYPE_UNKNOWN)
            return LHM_HINGE_STATUS_ERROR;
        if (currentMotionSequence.sequenceType() == MS_SEQ_TYPE_LANDING)
        {
            MotionSequenceStatusType status = currentMotionSequence.status();
            if (status == MS_SEQ_STATUS_COMPLETED)
            {
                return LHM_HINGE_STATUS_LANDING_POSITION_READY;
            }
            else if (status == MS_SEQ_STATUS_BUSY || status == MS_SEQ_STATUS_STAGE_COMPLETED)
            {
                return LHM_HINGE_STATUS_LANDING_POSITION_IN_TRANSITION;
            }
            else
            {
                return LHM_HINGE_STATUS_ERROR;
            }
        }
        else
            return LHM_HINGE_STATUS_ERROR;
    }
}

lhm_hook_status_t LHMController::getHookStatus()
{
    lhm_limit_switch_status_t top_ls = getTopLimitSwitchStatus();
    lhm_limit_switch_status_t bot_ls = getBotLimitSwitchStatus();
    LHM_DEBUG_PRINTF("LHMController::getHookStatus::top=[%d], bot=[%d]\n", top_ls, bot_ls);

    if (top_ls == LHM_LS_STATUS_ERROR || bot_ls == LHM_LS_STATUS_ERROR)
        return LHM_HOOK_STATUS_ERROR;
    if (top_ls == LHM_LS_STATUS_CLOSED && bot_ls == LHM_LS_STATUS_OPEN)
    {
        if (hookMotor.isTorqueOn() && hookMotionStatus == LHM_HOOK_STATUS_CLOSING)
            return LHM_HOOK_STATUS_CLOSING;
        return LHM_HOOK_STATUS_FULLY_OPEN;
    }
    if (top_ls == LHM_LS_STATUS_OPEN && bot_ls == LHM_LS_STATUS_CLOSED)
    {
        if (hookMotor.isTorqueOn() && hookMotionStatus == LHM_HOOK_STATUS_OPENNING)
            return LHM_HOOK_STATUS_OPENNING;
        return LHM_HOOK_STATUS_FULLY_CLOSED;
    }
    if (top_ls == LHM_LS_STATUS_CLOSED && bot_ls == LHM_LS_STATUS_CLOSED)
    {
        return LHM_HOOK_STATUS_ERROR;
    }
    if (top_ls == LHM_LS_STATUS_OFFLINE || bot_ls == LHM_LS_STATUS_OFFLINE || !hookMotor.isOnline())
    {
        return LHM_HOOK_STATUS_OFFLINE;
    }
    if (hookMotor.isTorqueOn())
    {
        return hookMotionStatus;
    }
    return LHM_HOOK_STATUS_LOOSE;
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
    currentMotionSequence = MotionSequence(MS_SEQ_TYPE_LANDING, motors, MOTION_SEQ_LANDING);
    return nextMotionSequence() == MS_EXE_RE_SUCCESSFUL;
}

bool LHMController::isAtLandingPosition()
{
    if (currentMotionSequence.sequenceType() == MS_SEQ_TYPE_LANDING)
    {
        return currentMotionSequence.status() == MS_SEQ_STATUS_COMPLETED;
    }
    else
    {
        return false;
    }
}

MotionSequenceStatusType LHMController::getMotionSequenceStatus()
{
    return currentMotionSequence.status();
}

MotionSequenceExecusionResultType LHMController::nextMotionSequence()
{    
    uint8_t s = getMotionSequenceStatus();
    if (s == MS_SEQ_STATUS_COMPLETED)
    {
        return MS_EXE_RE_SEQUENCE_ENDED;
    }
    else if (s == MS_SEQ_STATUS_BUSY)
    {
        return MS_EXE_RE_FAILED;
    }
    else if (s != MS_SEQ_STATUS_WAITING)
    {
        return MS_EXE_RE_ERROR;
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
    lhm_hook_status_t hStatus = getHookStatus();
    if (hStatus == LHM_HOOK_STATUS_FULLY_OPEN)
    {
        return true;
    }
    if (hStatus == LHM_HOOK_STATUS_OFFLINE || hStatus == LHM_HOOK_STATUS_ERROR)
    {
        return false;
    }
    bool result = hookMotor.setTorqueOn();
    result = result && hookMotor.setGoalVelocity(VELOCITY_HOOK_MOTOR_OPEN);
    if (result)
    {
        hookMotionStatus = LHM_HOOK_STATUS_OPENNING;
    }
    return result;
}

bool LHMController::closeHook()
{
    lhm_hook_status_t hStatus = getHookStatus();
    if (hStatus == LHM_HOOK_STATUS_FULLY_CLOSED)
    {
        return true;
    }
    if (hStatus == LHM_HOOK_STATUS_OFFLINE || hStatus == LHM_HOOK_STATUS_ERROR)
    {
        return false;
    }

    bool result = hookMotor.setTorqueOn();
    result = result && hookMotor.setGoalVelocity(VELOCITY_HOOK_MOTOR_CLOSE);
    if (result)
    {
        hookMotionStatus = LHM_HOOK_STATUS_CLOSING;
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
        hookMotionStatus = LHM_HOOK_STATUS_UNKNOWN;
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

lhm_limit_switch_status_t LHMController::getBotLimitSwitchStatus()
{
    return getLimitSwitchStatus(PIN_LIMIT_SWITCH_CLOSED_BOT, PIN_LIMIT_SWITCH_OPEN_BOT);
}

lhm_limit_switch_status_t LHMController::getTopLimitSwitchStatus()
{
    return getLimitSwitchStatus(PIN_LIMIT_SWITCH_CLOSED_TOP, PIN_LIMIT_SWITCH_OPEN_TOP);
}

lhm_limit_switch_status_t LHMController::getLimitSwitchStatus(uint8_t closed_pin, uint8_t open_pin, bool offlineRetry)
{
    bool closed_pin_on = digitalReadExt(closed_pin) == HIGH;
    bool open_pin_on = digitalReadExt(open_pin) == HIGH;
    lhm_limit_switch_status_t status;
    if (closed_pin_on && !open_pin_on)
        status = LHM_LS_STATUS_CLOSED;
    else if (!closed_pin_on && open_pin_on)
        status = LHM_LS_STATUS_OPEN;
    else if (closed_pin_on && open_pin_on)
        status = LHM_LS_STATUS_ERROR;
    else if (offlineRetry)
    {
        delay(10);
        status = getLimitSwitchStatus(closed_pin, open_pin, false);
    }
    else
    {
        status = LHM_LS_STATUS_OFFLINE;
    }
    LHM_DEBUG_PRINTF("LHMController::getLimitSwitchStatus::pin[%d & %d] -> %d\n", closed_pin, open_pin, (int)status);
    return status;
}

on_off_t LHMController::getPESensorStatus()
{
    if (digitalReadExt(PIN_PE_SENSOR) == HIGH)
    {
        return ON;
    }
    return OFF;
}