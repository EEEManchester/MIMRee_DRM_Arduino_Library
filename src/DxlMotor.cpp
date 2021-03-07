#include "DxlMotor.h"
using namespace ControlTableItem;

DXLMotor::DXLMotor(Dynamixel2Arduino &dxl, int motorId)
    : dxl(dxl), id(motorId)
{
}

bool DXLMotor::isOnline()
{
    return DXLMotor::dxl.ping(DXLMotor::id);
}

bool DXLMotor::isTorqueOn()
{
    return DXLMotor::dxl.getTorqueEnableStat(DXLMotor::id);
}

float DXLMotor::getCurrentPosition()
{
    return DXLMotor::dxl.getPresentPosition(DXLMotor::id);
}

float DXLMotor::getCurrentVelocity()
{
    return DXLMotor::dxl.getPresentVelocity(DXLMotor::id);
}

bool DXLMotor::isMoving()
{
    return DXLMotor::dxl.readControlTableItem(122, DXLMotor::id) == 1;
}

bool DXLMotor::isAtGoalPosition()
{
    return isAtPosition(DXLMotor::goalPosition);
}

bool DXLMotor::isAtPosition(float position)
{
    return position - getCurrentPosition() < 2;
}

bool DXLMotor::setOperatingMode(OperatingMode op)
{
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
    return result;
}

bool DXLMotor::setGoalVelocity(float velocity)
{
    bool result = setTorqueOn();
    result = result && DXLMotor::dxl.setGoalVelocity(DXLMotor::id, velocity);
    if (result)
    {
        DXLMotor::goalVelocity = velocity;
        DXLMotor::errorStatus = false;
    }
    else
    {
        DXLMotor::errorStatus = true;
    }
    return result;
}

bool DXLMotor::setGoalCurrent(float current)
{
    bool result = setTorqueOn();
    result = result && DXLMotor::dxl.setGoalCurrent(DXLMotor::id, current);
    if (result)
    {
        DXLMotor::goalCurrent = current;
        DXLMotor::errorStatus = false;
    }
    else
    {
        DXLMotor::errorStatus = true;
    }
    return result;
}

bool DXLMotor::setGoalPosition(float position)
{
    bool result = setTorqueOn();
    result = result && DXLMotor::dxl.writeControlTableItem(PROFILE_ACCELERATION, DXLMotor::id, PROFILE_ACCELERATION_VAL);
    result = result && DXLMotor::dxl.writeControlTableItem(PROFILE_VELOCITY, DXLMotor::id, PROFILE_VELOCITY_VAL);
    result = result && DXLMotor::dxl.setGoalPosition(DXLMotor::id, position);
    if (result)
    {
        DXLMotor::goalPosition = position;
        DXLMotor::errorStatus = false;
    }
    else
    {
        DXLMotor::errorStatus = true;
    }
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