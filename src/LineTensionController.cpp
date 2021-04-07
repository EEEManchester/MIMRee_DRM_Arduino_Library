#include "LineTensionController.h"

LineTensionController::LineTensionController(DXLMotor &ltcMotor)
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
    ltcMotor.dxl.torqueOff(ltcMotor.getId());
    _mode = TC_POWERED_OFF;
    return true;
}

bool LineTensionController::_detension()
{
    LineStatusType lsType = getLineStatus();
    if (lsType == LINE_IN_TENSION)
    {
        return runTillLineIsLoose(TC_VELOCITY_INITIAL, TC_TENSIONING_ROT_DIR * -1);
    }
    if (lsType == LINE_IN_TENSION_REVERSED)
    {
        return runTillLineIsLoose(TC_VELOCITY_INITIAL, TC_TENSIONING_ROT_DIR);
    }
    return true;
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
    DEBUG_SERIAL.println("LineTensionController::holdHome: Starting home sequence.");
    delay(2);
    LineStatusType lsType = getLineStatus();
    if (lsType == LINE_STATUS_ERROR || lsType == LINE_STATUS_UNKNOWN)
    {
        return false;
    }
    if (lsType == LINE_IN_TENSION_REVERSED || lsType == LINE_LOOSE)
    {
        DEBUG_SERIAL.println("LineTensionController::holdHome: Pickup line is loose or tensioned in reversed direction. Begin to return to holdHome.");
        delay(2);
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
    delay(2);
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

LineStatusType LineTensionController::getLineStatus(bool retryOnError)
{
    int lsHighNO = digitalRead(TC_PIN_LIMIT_SWITCH_HIGH_NO) == LOW;
    int lsHighNC = digitalRead(TC_PIN_LIMIT_SWITCH_HIGH_NC) == LOW;
    int lsLowNO = digitalRead(TC_PIN_LIMIT_SWITCH_LOW_NO) == LOW;
    int lsLowNC = digitalRead(TC_PIN_LIMIT_SWITCH_LOW_NC) == LOW;
    if (!lsHighNO && lsHighNC && !lsLowNO && lsLowNC)
    {
        return LINE_LOOSE;
    }
    if (lsHighNO && !lsHighNC && !lsLowNO && lsLowNC)
    {
        return LINE_IN_TENSION;
    }
    if (!lsHighNO && lsHighNC && lsLowNO && !lsLowNC)
    {
        return LINE_IN_TENSION_REVERSED;
    }
    if (retryOnError)
    {
        delay(50);
        return getLineStatus(false);
    }
    return LINE_STATUS_ERROR;
}

bool LineTensionController::runTillLineInTension(int32_t speed)
{
    LineStatusType lsType = getLineStatus();
    DEBUG_SERIAL.printf("LineTensionController::runTillLineInTension: getLineStatus is %d\n", lsType);
    if (lsType == LINE_IN_TENSION)
    {
        DEBUG_SERIAL.println("LineTensionController::runTillLineInTension: Pickup line is already in tension. Skip sequence.");
        delay(10);
        return true;
    }
    DEBUG_SERIAL.println("test");
    delay(10);
    bool result = ltcMotor.setOperatingMode(OP_VELOCITY);
    delay(10);
    result = result && ltcMotor.setTorqueOn();
    delay(10);
    result = result && ltcMotor.setGoalVelocity(speed * TC_TENSIONING_ROT_DIR);
    delay(10);
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
        lsType = getLineStatus();
    }
    result = ltcMotor.setTorqueOff();
    if (!result)
    {
        return false;
    }
    DEBUG_SERIAL.println("LineTensionController::runTillLineInTension: Pickup line is in tension.");
    return true;
}

bool LineTensionController::runTillLineIsLoose(int32_t speed, int8_t dir)
{
    LineStatusType lsType = getLineStatus();
    if (lsType == LINE_LOOSE)
    {
        DEBUG_SERIAL.println("LineTensionController::runTillLineIsLoose: Pickup line is already loose. Skip sequence.");
        return true;
    }
    DEBUG_SERIAL.printf("LineTensionController::runTillLineIsLoose: inputs - spd[%f] dir[%f] mul[%f]\n", (float)speed, (float)dir, (float)((float)speed*(float)dir));
    bool result = ltcMotor.setOperatingMode(OP_VELOCITY);
    result = result && ltcMotor.setTorqueOn();
    result = result && ltcMotor.setGoalVelocity(speed * dir);
    delay(2);
    if (!result)
    {
        return false;
    }
    lsType = getLineStatus();
    while (lsType != LINE_LOOSE)
    {
        // DEBUG_SERIAL.printf("LineTensionController::runTillLineIsLoose: Line status: %d\n", lsType);
        // if (dir == TC_TENSIONING_ROT_DIR)
        // {
        //     if (lsType != LINE_IN_TENSION_REVERSED)
        //     {
        //         return false;
        //     }
        // }
        // else
        // {
        //     if (lsType != LINE_IN_TENSION)
        //     {
        //         return false;
        //     }
        // }
        lsType = getLineStatus();
    }
    result = ltcMotor.setTorqueOff();
    if (!result)
    {
        return false;
    }
    DEBUG_SERIAL.println("LineTensionController::runTillLineIsLoose: Pickup line is loose.");
    return true;
}

bool LineTensionController::softEmergencyStop(char *message)
{
    DEBUG_SERIAL.println("LineTensionController::softEmergencyStop: Emergency stop requested.");
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
