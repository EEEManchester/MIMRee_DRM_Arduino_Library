#include "DxlMotor.h"
using namespace ControlTableItem;

DXLMotor::DXLMotor(Dynamixel2Arduino &dxl, int motorId, LHMMessage &lhmMessage)
    : dxl(dxl), id(motorId), lhmMessage(lhmMessage)
{
}

bool DXLMotor::reboot()
{
    bool result = DXLMotor::dxl.reboot(DXLMotor::id);
    DXLMotor::lhmMessage.debugSerial.printf("DXLMotor::reboot:: id[%d] = %d\n", DXLMotor::id, result);
    return result;
}

bool DXLMotor::isOnline()
{
    bool result = DXLMotor::dxl.ping(DXLMotor::id);
    DXLMotor::lhmMessage.debugSerial.printf("DXLMotor::isOnline:: id[%d] = %d\n", DXLMotor::id, result);
    return result;
}

bool DXLMotor::isTorqueOn()
{
    bool result = DXLMotor::dxl.getTorqueEnableStat(DXLMotor::id);
    DXLMotor::lhmMessage.debugSerial.printf("DXLMotor::isTorqueOn:: id[%d] = %d\n", DXLMotor::id, result);
    return result;
}

float DXLMotor::getCurrentPosition()
{
    float result = DXLMotor::dxl.getPresentPosition(DXLMotor::id);
    DXLMotor::lhmMessage.debugSerial.printf("DXLMotor::getCurrentPosition:: id[%d] = %f\n", DXLMotor::id, result);
    return result;
}

float DXLMotor::getCurrentVelocity()
{
    float result = DXLMotor::dxl.getPresentVelocity(DXLMotor::id);
    DXLMotor::lhmMessage.debugSerial.printf("DXLMotor::getCurrentVelocity:: id[%d] = %f\n", DXLMotor::id, result);
    return result;
}

bool DXLMotor::isMoving()
{
    int32_t result = DXLMotor::dxl.readControlTableItem(MOVING, DXLMotor::id);
    DXLMotor::lhmMessage.debugSerial.printf("DXLMotor::isMoving:: id[%d] = %d\n", DXLMotor::id, result);
    return result == 1;
    // if (!isTorqueOn())
    // {
    //     return false;
    // }
    // OperatingMode opMode = getCurrentOperatingMode();
    // if (opMode == OP_CURRENT)
    // {
    //     return true;
    // }
    // bool isStationary = getCurrentVelocity() < MOVING_THRESHOLD_VELOCITY;
    // if (opMode == OP_VELOCITY)
    // {
    //     return isStationary;
    // }
    // if (opMode == OP_POSITION)
    // {
    //     if (!isStationary)
    //     {
    //         return true;
    //     }
    //     return abs(getCurrentPosition() - getCurrentGoalPosition()) > POSITION_TOLERANCE;
    // }
}

bool DXLMotor::isAtGoalPosition()
{
    return isAtPosition(DXLMotor::goalPosition);
}

bool DXLMotor::isAtPosition(float position)
{
    return position - getCurrentPosition() < POSITION_TOLERANCE;
}

bool DXLMotor::setOperatingMode(OperatingMode op)
{
    // For safety concerns, always torque off before setting operating mode.
    if (isTorqueOn())
    {
        setTorqueOff();
    }

    bool result = DXLMotor::dxl.setOperatingMode(DXLMotor::id, op);
    if (result)
    {
        DXLMotor::operationMode = op;
        DXLMotor::errorStatus = false;
    }
    else
    {
        DXLMotor::errorStatus = true;
    }
    DXLMotor::lhmMessage.debugSerial.printf("DXLMotor::setOperatingMode:: id[%d] -> [%d] = %d\n", DXLMotor::id, op, result);
    return result;
}

bool DXLMotor::setTorqueOff()
{
    bool result = DXLMotor::dxl.torqueOff(DXLMotor::id);
    if (result)
    {
        DXLMotor::errorStatus = false;
        setLED(false);
    }
    else
    {
        DXLMotor::errorStatus = true;
    }
    DXLMotor::lhmMessage.debugSerial.printf("DXLMotor::setTorqueOff:: id[%d] = %d\n", DXLMotor::id, result);
    return result;
}

bool DXLMotor::setTorqueOn()
{
    bool result = DXLMotor::dxl.torqueOn(DXLMotor::id);
    if (result)
    {
        DXLMotor::errorStatus = false;
        setLED(true);
    }
    else
    {
        DXLMotor::errorStatus = true;
    }
    DXLMotor::lhmMessage.debugSerial.printf("DXLMotor::setTorqueOn:: id[%d] = %d\n", DXLMotor::id, result);
    return result;
}

bool DXLMotor::setVelocityProfile(int val)
{
    bool result = DXLMotor::dxl.writeControlTableItem(PROFILE_VELOCITY, DXLMotor::id, val);
    DXLMotor::lhmMessage.debugSerial.printf("DXLMotor::setVelocityProfile:: id[%d] = %d\n", DXLMotor::id, result);
    return result;
}

bool DXLMotor::setAccelerationProfile(int val)
{
    bool result = DXLMotor::dxl.writeControlTableItem(PROFILE_ACCELERATION, DXLMotor::id, val);
    DXLMotor::lhmMessage.debugSerial.printf("DXLMotor::setAccelerationProfile:: id[%d] = %d\n", DXLMotor::id, result);
    return result;
}

bool DXLMotor::setGoalVelocity(float velocity)
{    
    bool result = DXLMotor::dxl.setGoalVelocity(DXLMotor::id, velocity);
    if (result)
    {
        DXLMotor::goalVelocity = velocity;
        DXLMotor::errorStatus = false;
    }
    else
    {
        DXLMotor::errorStatus = true;
    }
    DXLMotor::lhmMessage.debugSerial.printf("DXLMotor::setGoalVelocity:: id[%d] -> [%f] = %d\n", DXLMotor::id, velocity, result);
    return result;
}

bool DXLMotor::setGoalCurrent(float current)
{
    bool result = DXLMotor::dxl.setGoalCurrent(DXLMotor::id, current);
    if (result)
    {
        DXLMotor::goalCurrent = current;
        DXLMotor::errorStatus = false;
    }
    else
    {
        DXLMotor::errorStatus = true;
    }
    DXLMotor::lhmMessage.debugSerial.printf("DXLMotor::setGoalCurrent:: id[%d] -> [%f] = %d\n", DXLMotor::id, current, result);
    return result;
}

bool DXLMotor::setGoalPosition(float position)
{
    bool result = DXLMotor::dxl.setGoalPosition(DXLMotor::id, position);
    if (result)
    {
        DXLMotor::goalPosition = position;
        DXLMotor::errorStatus = false;
    }
    else
    {
        DXLMotor::errorStatus = true;
    }
    DXLMotor::lhmMessage.debugSerial.printf("DXLMotor::setGoalPosition:: id[%d] -> [%f] = %d\n", DXLMotor::id, position, result);
    return result;
}

void DXLMotor::setLED(bool setOn)
{
    if (setOn)
    {
        DXLMotor::dxl.ledOn(DXLMotor::id);
    }
    else
    {
        DXLMotor::dxl.ledOff(DXLMotor::id);
    }
}

void DXLMotor::flashLED(int times, unsigned long interval)
{
    for (int i = 0; i < times; i++)
    {
        setLED(true);
        delay(interval);
        setLED(false);
        delay(interval);
    }
}