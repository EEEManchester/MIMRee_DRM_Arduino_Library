#include "LinkHookModuleController.h"

LHMController::LHMController(HardwareSerial &servoSerial, USBSerial &debugSerial)
    : dxl(Dynamixel2Arduino(servoSerial, PIN_DXL_DIR)),
      hookMotor(DXLMotor(dxl, MOTOR_ID_HOOK)),
      hingeMotorX(DXLMotor(dxl, MOTOR_ID_HINGE_X)),
      hingeMotorY(DXLMotor(dxl, MOTOR_ID_HINGE_Y)),
      lhmMessage(debugSerial)
{
}

void LHMController::initiateDxl()
{
    LHMController::dxl.begin(DXL_BAUD_RATE);
    LHMController::dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    LHMController::hingeMotorX.setLED(true);
    LHMController::hingeMotorY.setLED(true);
    LHMController::hookMotor.setLED(true);
    delay(3000);
    LHMController::hingeMotorX.setLED(false);
    LHMController::hingeMotorY.setLED(false);
    LHMController::hookMotor.setLED(false);

    LHMController::hookMotor.setOperatingMode(OP_VELOCITY);
    LHMController::hingeMotorX.setTorqueOn();
    LHMController::hingeMotorY.setTorqueOn();
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

        bool moving_x = LHMController::hingeMotorX.isMoving();
        bool moving_y = LHMController::hingeMotorY.isMoving();
        if (moving_x || moving_y)
            return HingeStatus::LANDING_POSITION_IN_TRANSITION;
        else
            return HingeStatus::LANDING_POSITION_READY;
    }
}

HookStatus LHMController::getHookStatus()
{
    LimitSwitchStatus top_ls = getTopLimitSwitchStatus();
    LimitSwitchStatus bot_ls = getBotLimitSwitchStatus();
    LHMController::lhmMessage.sendDebugMessage("LHMController::getHookStatus::top=[%i], bot=[%i]", top_ls, bot_ls);
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
    if (!LHMController::hookMotor.isMoving())
    {
        return HookStatus::LOOSE;
    }
    return LHMController::hookMotionStatus;
}

bool LHMController::isEngaged()
{
    return getPESensorStatus() == OnOffStatus::ON;
}

bool LHMController::setSwingReductionMode()
{
    bool result = LHMController::hingeMotorX.setOperatingMode(OP_CURRENT);
    result = result && LHMController::hingeMotorX.setGoalCurrent(0);
    result = LHMController::hingeMotorY.setOperatingMode(OP_CURRENT) && result;
    result = result && LHMController::hingeMotorY.setGoalCurrent(0);
    LHMController::lhmMessage.sendCommandFeedback(HingeCommand::SWING_REDUCTION, result);
    return result;
}

bool LHMController::setLandingPosition()
{
    bool result = LHMController::hingeMotorX.setOperatingMode(OP_POSITION);
    result = result && LHMController::hingeMotorX.setGoalPosition(HINGE_X_VAL_LANDING_POISITION);
    LHMController::lhmMessage.sendCommandFeedback(HingeCommand::LANDING, result);
    return result;
}

bool LHMController::isAtLandingPosition()
{
    return LHMController::hingeMotorX.isAtPosition(HINGE_X_VAL_LANDING_POISITION);
}

bool LHMController::setTakeoffMode()
{
    bool result;
    result = LHMController::hingeMotorX.setTorqueOff();
    result = LHMController::hingeMotorY.setTorqueOff() && result;
    LHMController::lhmMessage.sendCommandFeedback(HingeCommand::TAKE_OFF, result);
    return result;
}

bool LHMController::stopHingeMotor()
{
    bool result = LHMController::hingeMotorX.setTorqueOff();
    result = LHMController::hingeMotorY.setTorqueOff() && result;
    LHMController::lhmMessage.sendCommandFeedback(HingeCommand::POWER_OFF, result);
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
    LHMController::lhmMessage.sendDebugMessage("LHMController::getLimitSwitchStatus");
    if (digitalReadExt(closed_pin) == HIGH)
    {
        return LimitSwitchStatus::CLOSED;
    }
    if (digitalReadExt(open_pin) == HIGH)
    {
        return LimitSwitchStatus::OPEN;
    }
    return LimitSwitchStatus::OFFLINE;
}

OnOffStatus LHMController::getPESensorStatus()
{
    LHMController::lhmMessage.sendDebugMessage("LHMController::getPESensorStatus");
    if (digitalReadExt(PIN_PE_SENSOR) == HIGH)
    {
        return OnOffStatus::ON;
    }
    return OnOffStatus::OFF;
}

int LHMController::digitalReadExt(int pin)
{
    int state = digitalRead(pin);
    LHMController::lhmMessage.sendDebugMessage("LHMController::digitalReadExt::pin[%i]=%i", PIN_PE_SENSOR, state);
}