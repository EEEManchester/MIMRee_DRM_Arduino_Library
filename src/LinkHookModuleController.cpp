#include "LinkHookModuleController.h"

LHMController::LHMController(HardwareSerial &servoSerial, COM_SERIAL_CLASS &comSerial, DEBUG_SERIAL_CLASS &debugSerial)
    : dxl(Dynamixel2Arduino(servoSerial, PIN_DXL_DIR)),
      lhmMessage(comSerial, debugSerial),
      hookMotor(DXLMotor(dxl, MOTOR_ID_HOOK, lhmMessage)),
      hingeMotorX(DXLMotor(dxl, MOTOR_ID_HINGE_X, lhmMessage)),
      hingeMotorY(DXLMotor(dxl, MOTOR_ID_HINGE_Y, lhmMessage))
{
}

void LHMController::initiateDxl()
{
    LHMController::dxl.begin(DXL_BAUD_RATE);
    LHMController::dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    LHMController::hookMotor.setOperatingMode(OP_VELOCITY);
    LHMController::hingeMotorX.setTorqueOff();
    LHMController::hingeMotorY.setTorqueOff();

    pinMode(PIN_LIMIT_SWITCH_CLOSED_BOT, INPUT_PULLDOWN);
    pinMode(PIN_LIMIT_SWITCH_CLOSED_TOP, INPUT_PULLDOWN);
    pinMode(PIN_LIMIT_SWITCH_OPEN_BOT, INPUT_PULLDOWN);
    pinMode(PIN_LIMIT_SWITCH_OPEN_TOP, INPUT_PULLDOWN);
}

HingeStatus LHMController::getHingeStatus()
{
    if (!LHMController::hingeMotorX.isOnline() || !LHMController::hingeMotorY.isOnline())
    {
        return HingeStatus::OFFLINE;
    }
    bool torque_x = LHMController::hingeMotorX.isTorqueOn();
    bool torque_y = LHMController::hingeMotorY.isTorqueOn();
    if (!torque_x)
    {
        if (!torque_y)
            return HingeStatus::TAKEOFF_MODE;
        else
            return HingeStatus::ERROR;
    }
    OperatingMode op_x = LHMController::hingeMotorX.getLastSetOperatingMode();
    OperatingMode op_y = LHMController::hingeMotorY.getLastSetOperatingMode();
    if (op_x == OP_CURRENT)
    {
        if (op_y == OP_CURRENT)
            return HingeStatus::SWING_REDUCTION;
        else
            return HingeStatus::ERROR;
    }
    if (op_x == OP_POSITION)
    {
        if (!op_y == OP_POSITION)
            return HingeStatus::ERROR;
        if (LHMController::hingeMotorX.getLastSetGoalPosition() != HINGE_X_VAL_LANDING_POISITION || LHMController::hingeMotorY.getLastSetGoalPosition() != HINGE_Y_VAL_LANDING_POISITION)
            return HingeStatus::ERROR;

        if (LHMController::hingeMotorX.isAtGoalPosition() && LHMController::hingeMotorY.isAtGoalPosition())
            return HingeStatus::LANDING_POSITION_READY;
        else
            return HingeStatus::LANDING_POSITION_IN_TRANSITION;
    }
}

HookStatus LHMController::getHookStatus()
{
    LimitSwitchStatus top_ls = getTopLimitSwitchStatus();
    LimitSwitchStatus bot_ls = getBotLimitSwitchStatus();
    LHMController::lhmMessage.debugSerial.printf("LHMController::getHookStatus::top=[%d], bot=[%d]\n", top_ls, bot_ls);
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
    if (LHMController::hookMotor.isMoving())
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
    bool result = LHMController::hingeMotorX.setOperatingMode(OP_CURRENT);
    result = result && LHMController::hingeMotorX.setTorqueOn();
    result = result && LHMController::hingeMotorX.setGoalCurrent(0);
    result = result && LHMController::hingeMotorY.setOperatingMode(OP_CURRENT);
    result = result && LHMController::hingeMotorY.setTorqueOn();
    result = result && LHMController::hingeMotorY.setGoalCurrent(0);
    LHMController::lhmMessage.debugSerial.printf("LHMController::setSwingReductionMode: %s\n", result ? "Successful" : "Failed");
    LHMController::lhmMessage.sendCommandFeedback(CommandType::HINGE_SWING_REDUCTION, result);
    return result;
}

bool LHMController::setLandingPosition()
{
    bool result = LHMController::hingeMotorX.setOperatingMode(OP_POSITION);
    result = result && LHMController::hingeMotorX.setAccelerationProfile(PROFILE_ACCELERATION_VAL);
    result = result && LHMController::hingeMotorX.setVelocityProfile(PROFILE_VELOCITY_VAL);
    result = result && LHMController::hingeMotorX.setTorqueOn();
    result = result && LHMController::hingeMotorX.setGoalPosition(HINGE_X_VAL_LANDING_POISITION);
    result = result && LHMController::hingeMotorY.setOperatingMode(OP_POSITION);
    result = result && LHMController::hingeMotorY.setAccelerationProfile(PROFILE_ACCELERATION_VAL);
    result = result && LHMController::hingeMotorY.setVelocityProfile(PROFILE_VELOCITY_VAL);
    result = result && LHMController::hingeMotorY.setTorqueOn();
    result = result && LHMController::hingeMotorY.setGoalPosition(HINGE_Y_VAL_LANDING_POISITION);
    LHMController::lhmMessage.debugSerial.printf("LHMController::setLandingPosition: %s\n", result ? "Successful" : "Failed");
    LHMController::lhmMessage.sendCommandFeedback(CommandType::HINGE_LANDING, result);
    return result;
}

bool LHMController::isAtLandingPosition()
{
    return LHMController::hingeMotorX.isAtPosition(HINGE_X_VAL_LANDING_POISITION);
}

bool LHMController::setTakeoffMode()
{
    bool result = LHMController::hingeMotorX.setTorqueOff();
    result = result && LHMController::hingeMotorY.setTorqueOff();
    LHMController::lhmMessage.debugSerial.printf("LHMController::setTakeoffMode: %s\n", result ? "Successful" : "Failed");
    LHMController::lhmMessage.sendCommandFeedback(CommandType::HINGE_TAKE_OFF, result);
    return result;
}

bool LHMController::stopHingeMotor()
{
    bool result = LHMController::hingeMotorX.setTorqueOff();
    result = result && LHMController::hingeMotorY.setTorqueOff();
    LHMController::lhmMessage.debugSerial.printf("LHMController::stopHingeMotor: %s\n", result ? "Successful" : "Failed");
    LHMController::lhmMessage.sendCommandFeedback(CommandType::HINGE_POWER_OFF, result);
    return result;
}

void LHMController::openHook()
{
    if (getHookStatus() == HookStatus::FULLY_OPEN)
    {
        return;
    }

    //TODO make this none blocking
    bool result = LHMController::hookMotor.setGoalVelocity(VELOCITY_HOOK_MOTOR_OPEN);
    if (!result)
    {
        return;
    }
    LHMController::hookMotionStatus = HookStatus::OPENNING;
    while (getHookStatus() != HookStatus::FULLY_OPEN)
    {
        delay(50);
    }
    LHMController::hookMotor.setGoalVelocity(0);
    delay(100);
    LHMController::hookMotor.setTorqueOff();
    LHMController::hookMotionStatus = HookStatus::UNKNOWN;
}

void LHMController::closeHook()
{
    if (getHookStatus() == HookStatus::FULLY_CLOSED)
    {
        return;
    }

    //TODO make this none blocking
    bool result = LHMController::hookMotor.setGoalVelocity(VELOCITY_HOOK_MOTOR_CLOSE);
    if (!result)
    {
        return;
    }
    LHMController::hookMotionStatus = HookStatus::CLOSING;
    while (getHookStatus() != HookStatus::FULLY_CLOSED)
    {
        delay(50);
    }
    LHMController::hookMotor.setGoalVelocity(0);
    delay(100);
    LHMController::hookMotor.setTorqueOff();
    LHMController::hookMotionStatus = HookStatus::UNKNOWN;
}

void LHMController::stopHookMotor()
{
    LHMController::hookMotor.setGoalVelocity(0);
    delay(100);
    LHMController::hookMotor.setTorqueOff();
    LHMController::hookMotionStatus = HookStatus::UNKNOWN;
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
    LHMController::lhmMessage.debugSerial.printf("LHMController::getLimitSwitchStatus::pin[%d & %d] -> %d\n", closed_pin, open_pin, (int)status);
    return status;
}

OnOffStatus LHMController::getPESensorStatus()
{
    LHMController::lhmMessage.debugSerial.println("LHMController::getPESensorStatus");
    if (digitalReadExt(PIN_PE_SENSOR) == HIGH)
    {
        return OnOffStatus::ON;
    }
    return OnOffStatus::OFF;
}

int LHMController::digitalReadExt(int pin)
{
    int state = digitalRead(pin);
    LHMController::lhmMessage.debugSerial.printf("LHMController::digitalReadExt::pin[%d]=%d\n", pin, state);
    return state;
}