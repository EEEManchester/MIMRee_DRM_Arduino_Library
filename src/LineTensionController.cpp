#include "LineTensionController.h"

LineTensionController::LineTensionController(DXLMotor ltcMotor)
    : ltcMotor(ltcMotor)
{
}

bool LineTensionController::run(LineTensionControlCommandType command)
{
    switch (command)
    {
    case TC_DETENSION:
        return detension();
    case TC_HOME:
        return holdHome();
    case TC_PREPARE_FOR_ENGAGEMENT:
        return prepareEngagement();
    case TC_POWER_OFF:
        return powerOff();
    case TC_HOLD_POSITION:
        return holdPosition();
    default:
        return false;
    }
}

bool LineTensionController::holdPosition()
{
    bool result = ltcMotor.setOperatingMode(OP_EXTENDED_POSITION);
    if (!result)
    {
        return false;
    }
    return ltcMotor.isTorqueOn();
}

bool LineTensionController::powerOff()
{
    bool result = ltcMotor.setTorqueOff();
    if (result)
    {
        _mode = TC_POWERED_OFF;
    }
    return result;
}

bool LineTensionController::detension()
{
    if (!_detension())
    {
        softEmergencyStop("LineTensionController::detension: Fail to detension. Error occurred.");
        return false;
    }
    ltcMotor.setTorqueOff();
    _mode = TC_POWERED_OFF;
    return true;
}

bool LineTensionController::_detension()
{
    LineStatusType lsType = lineStatus();
    if (lsType == LINE_IN_TENSION)
    {
        return runTillLineIsLoose(TC_VELOCITY_INITIAL, TC_TENSIONING_ROT_DIR * -1);
    }
    if (lsType == LINE_IN_TENSION_REVERSED)
    {
        return runTillLineIsLoose(TC_VELOCITY_INITIAL, TC_TENSIONING_ROT_DIR);
    }
}

bool LineTensionController::holdHome()
{
    if (!_home())
    {
        softEmergencyStop("LineTensionController::holdHome: Fail to return to holdHome. Error occurred.");
        return false;
    }
    bool result = ltcMotor.setOperatingMode(OP_EXTENDED_POSITION);
    result = result && ltcMotor.isTorqueOn();
    if (!result)
    {
        softEmergencyStop("LineTensionController::holdHome: Home acquired but not able to hold position. Error occurred.");
        return false;
    }
    _mode = TC_POSITION_HOLDING;
    return true;
}

bool LineTensionController::_home()
{
    bool result = ltcMotor.setOperatingMode(OP_VELOCITY);
    result = result && ltcMotor.isTorqueOn();
    LineStatusType lsType = lineStatus();
    if (lsType == LINE_STATUS_ERROR || lsType == LINE_STATUS_UNKNOWN)
    {
        return false;
    }
    if (lsType == LINE_IN_TENSION_REVERSED || lsType == LINE_LOOSE)
    {
        DEBUG_SERIAL.println("LineTensionController::holdHome: Pickup line is loose or tensioned in reversed direction. Begin to return to holdHome.");
        if (!runTillLineInTension(TC_VELOCITY_MAX))
        {
            return false;
        }
    }
    if (!runTillLineIsLoose(TC_VELOCITY_INITIAL, TC_TENSIONING_ROT_DIR * -1))
    {
        return false;
    }
    DEBUG_SERIAL.println("LineTensionController::holdHome: Home position acquired successfully.");
    return true;
}

bool LineTensionController::prepareEngagement()
{
    if (!_prepareEngagement())
    {
        softEmergencyStop("LineTensionController::prepareEngagement: Fail to prepare for engagement. Error occurred.");
        return false;
    }
    bool result = ltcMotor.setTorqueOff();
    if (!result)
    {
        return softEmergencyStop("LineTensionController::prepareEngagement: In engagement stanby position but fail to turn off motor. Error occurred.");
    }
    return true;
}

bool LineTensionController::_prepareEngagement()
{
    bool result = _home();
    if (!result)
    {
        return false;
    }

    result = ltcMotor.setOperatingMode(OP_EXTENDED_POSITION);
    result = result && ltcMotor.setTorqueOn();
    float pos = ltcMotor.getCurrentPosition();
    result = result && ltcMotor.setGoalPosition((pos + TC_LOOSE_LINE_ROT_COUNT * TC_SERVO_ENCODER_BIT) * TC_TENSIONING_ROT_DIR * -1);
    if (!result)
    {
        return false;
    }
    DEBUG_SERIAL.println("LineTensionController::prepareEngagement: Start to loose pickup line.");

    while (ltcMotor.isMoving())
    {
        delay(200);
    }
    if (!ltcMotor.isAtGoalPosition(10))
    {
        return false;
    }
    DEBUG_SERIAL.println("LineTensionController::prepareEngagement: Sequence completed with success.");
    return true;
}

LineStatusType LineTensionController::lineStatus()
{
    int lsHighNO = digitalRead(TC_PIN_LIMIT_SWITCH_HIGH_NO) == LOW;
    int lsHighNC = digitalRead(TC_PIN_LIMIT_SWITCH_HIGH_NC) == LOW;
    int lsLowNO = digitalRead(TC_PIN_LIMIT_SWITCH_LOW_NO) == LOW;
    int lsLowNC = digitalRead(TC_PIN_LIMIT_SWITCH_LOW_NC) == LOW;
    if (lsHighNO && !lsHighNC && lsLowNO && !lsLowNC)
    {
        return LINE_LOOSE;
    }
    if (!lsHighNO && lsHighNC && lsLowNO && !lsLowNC)
    {
        return LINE_IN_TENSION;
    }
    if (lsHighNO && !lsHighNC && !lsLowNO && lsLowNC)
    {
        return LINE_IN_TENSION_REVERSED;
    }
    return LINE_STATUS_ERROR;
}

bool LineTensionController::runTillLineInTension(uint32_t speed)
{
    LineStatusType lsType = lineStatus();
    if (lsType == LINE_IN_TENSION)
    {
        DEBUG_SERIAL.println("LineTensionController::runTillLineInTension: Pickup line is already in tension. Skip sequence.");
        return true;
    }
    bool result = ltcMotor.setOperatingMode(OP_VELOCITY);
    result = result && ltcMotor.setTorqueOn();
    result = result && ltcMotor.setGoalVelocity(speed * TC_TENSIONING_ROT_DIR);
    if (!result)
    {
        return false;
    }
    while (lsType != LINE_IN_TENSION)
    {
        if (lsType != LINE_LOOSE)
        {
            return false;
        }
        delay(5);
        lsType = lineStatus();
    }
    result = ltcMotor.setTorqueOff();
    if (!result)
    {
        return false;
    }
    DEBUG_SERIAL.println("LineTensionController::runTillLineInTension: Pickup line is in tension.");
    return true;
}

bool LineTensionController::runTillLineIsLoose(uint32_t speed, uint8_t dir)
{
    LineStatusType lsType = lineStatus();
    if (lsType == LINE_IN_TENSION)
    {
        DEBUG_SERIAL.println("LineTensionController::runInReverseTillLineIsLoose: Pickup line is already loose. Skip sequence.");
        return true;
    }
    bool result = ltcMotor.setOperatingMode(OP_VELOCITY);
    result = result && ltcMotor.setTorqueOn();
    result = result && ltcMotor.setGoalVelocity(speed * dir);
    if (!result)
    {
        return false;
    }
    while (lsType != LINE_LOOSE)
    {
        if (dir == TC_TENSIONING_ROT_DIR)
        {
            if (lsType != LINE_IN_TENSION_REVERSED)
            {
                return false;
            }
        }
        else
        {
            if (lsType != LINE_IN_TENSION)
            {
                return false;
            }
        }
        lsType = lineStatus();
    }
    result = ltcMotor.setTorqueOff();
    if (!result)
    {
        return false;
    }
    DEBUG_SERIAL.println("LineTensionController::runInReverseTillLineIsLoose: Pickup line is loose.");
    return true;
}

bool LineTensionController::softEmergencyStop(char *message)
{
    ltcMotor.setTorqueOff();
    bool result = ltcMotor.reboot();
    DEBUG_SERIAL.println(message);
    if (!result)
    {
        _mode = TC_ERROR;
        DEBUG_SERIAL.println("LineTensionController::softEmergencyStop: Fail to reboot TC motor.");
        return false;
    }
    _mode = TC_POWERED_OFF;
    DEBUG_SERIAL.println("LineTensionController::softEmergencyStop: TC motor has been rebooted.");
    return true;
}
