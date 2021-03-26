#include "DxlMotor.h"
using namespace ControlTableItem;

DXLMotor::DXLMotor(Dynamixel2Arduino &dxl, uint8_t motorId, DEBUG_SERIAL_CLASS &debugSerial)
    : dxl(dxl), id(motorId), debugSerial(debugSerial)
{
}

bool DXLMotor::reboot()
{
    bool result = repeatCOM<uint8_t, uint32_t>(&Dynamixel2Arduino::reboot, id, 10U);
    debugSerial.printf("DXLMotor::reboot:: id[%d] = %d\n", id, result);
    lastSetOperatingMode = UNKNOWN_OP;
    resetOPRelatedParamRecords();
    return result;
}

bool DXLMotor::isOnline()
{
    bool result = repeatCOM<uint8_t>(&Dynamixel2Arduino::ping, id);
    debugSerial.printf("DXLMotor::isOnline:: id[%d][ping] = %d\n", id, result);
    bool result2 = isTorqueOn();
    return result || result2;
}

bool DXLMotor::isTorqueOn()
{
    bool result = repeatCOM<uint8_t>(&Dynamixel2Arduino::getTorqueEnableStat, id);
    debugSerial.printf("DXLMotor::isTorqueOn:: id[%d] = %d\n", id, result);
    return result;
}

float DXLMotor::getCurrentPosition()
{
    float result = dxl.getPresentPosition(id);
    debugSerial.printf("DXLMotor::getCurrentPosition:: id[%d] = %f\n", id, result);
    return result;
}

float DXLMotor::getCurrentVelocity()
{
    float result = dxl.getPresentVelocity(id);
    debugSerial.printf("DXLMotor::getCurrentVelocity:: id[%d] = %f\n", id, result);
    return result;
}

bool DXLMotor::isMoving()
{
    int32_t result = repeatCOM<uint8_t, uint8_t, uint32_t>(&Dynamixel2Arduino::readControlTableItem, MOVING, id, 100U);
    debugSerial.printf("DXLMotor::isMoving:: id[%d] = %d\n", id, result);
    return result == 1;
}

bool DXLMotor::isAtGoalPosition(float positionTolerance)
{
    return isAtPosition(lastSetGoalPosition, positionTolerance);
}

bool DXLMotor::isAtPosition(float position, float positionTolerance)
{
    if (positionTolerance < 0)
    {
        positionTolerance = 5;
    }
    float diff = abs(position - getCurrentPosition());
    bool result = diff < positionTolerance;
    debugSerial.printf("DXLMotor::isAtPosition:: id[%d] = %d | diff=%f\n", id, result, diff);
    return result;
}

bool DXLMotor::setOperatingMode(OperatingMode op)
{
    if (lastSetOperatingMode == op)
    {
        return true;
    }
    // For safety concerns, always torque off before setting operating mode.
    if (isTorqueOn())
    {
        setTorqueOff();
    }
    bool result = repeatCOM<uint8_t, uint8_t>(&Dynamixel2Arduino::setOperatingMode, id, op);
    if (result)
    {
        lastSetOperatingMode = op;
        errorStatus = false;
    }
    else
    {
        errorStatus = true;
    }
    debugSerial.printf("DXLMotor::setOperatingMode:: id[%d] -> [%d] = %d\n", id, op, result);
    resetOPRelatedParamRecords();
    return result;
}

bool DXLMotor::setTorqueOff()
{
    bool result = repeatCOM<uint8_t>(&Dynamixel2Arduino::torqueOff, id);
    if (result)
    {
        errorStatus = false;
        setLED(false);
        lastSetGoalPosition = -999;
        lastSetGoalVelocity = -999;
        lastSetGoalCurrent = -999;
    }
    else
    {
        errorStatus = true;
    }
    debugSerial.printf("DXLMotor::setTorqueOff:: id[%d] = %d\n", id, result);
    return result;
}

bool DXLMotor::setTorqueOn()
{
    bool result = repeatCOM<uint8_t>(&Dynamixel2Arduino::torqueOn, id);
    if (result)
    {
        errorStatus = false;
        setLED(true);
    }
    else
    {
        errorStatus = true;
    }
    Serial.printf("DXLMotor::setTorqueOn:: id[%d] = %d\n", id, result);
    return result;
}

bool DXLMotor::setVelocityProfile(int32_t val)
{
    if (val == lastSetVelocityProfile)
    {
        return true;
    }
    bool result = repeatCOM<uint8_t, uint8_t, int32_t, uint32_t>(&Dynamixel2Arduino::writeControlTableItem, PROFILE_VELOCITY, id, val, 100);
    Serial.printf("DXLMotor::setVelocityProfile:: id[%d] = %d\n", id, result);
    if (result)
    {
        lastSetVelocityProfile = val;
    }
    return result;
}

bool DXLMotor::setAccelerationProfile(int32_t val)
{
    if (val == lastSetAccelerationProfile)
    {
        return true;
    }
    bool result = repeatCOM<uint8_t, uint8_t, int32_t, uint32_t>(&Dynamixel2Arduino::writeControlTableItem, PROFILE_ACCELERATION, id, val, 100);
    debugSerial.printf("DXLMotor::setAccelerationProfile:: id[%d] = %d\n", id, result);
    if (result)
    {
        lastSetAccelerationProfile = val;
    }
    return result;
}

bool DXLMotor::setGoalVelocity(float velocity)
{
    if (abs(velocity - lastSetGoalVelocity) < 0.000001)
    {
        return true;
    }
    bool result = repeatCOM<uint8_t, float, uint8_t>(&Dynamixel2Arduino::setGoalVelocity, id, velocity, 0U);
    if (result)
    {
        lastSetGoalVelocity = velocity;
        errorStatus = false;
    }
    else
    {
        errorStatus = true;
    }
    debugSerial.printf("DXLMotor::setGoalVelocity:: id[%d] -> [%f] = %d\n", id, velocity, result);
    return result;
}

bool DXLMotor::setGoalCurrent(float current)
{
    if (abs(current - lastSetGoalCurrent) < 0.000001)
    {
        return true;
    }
    bool result = repeatCOM<uint8_t, float, uint8_t>(&Dynamixel2Arduino::setGoalCurrent, id, current, 0U);
    if (result)
    {
        lastSetGoalCurrent = current;
        errorStatus = false;
    }
    else
    {
        errorStatus = true;
    }
    debugSerial.printf("DXLMotor::setGoalCurrent:: id[%d] -> [%f] = %d\n", id, current, result);
    return result;
}

bool DXLMotor::setGoalPosition(float position)
{
    if (abs(position - lastSetGoalPosition) < 0.000001)
    {
        return true;
    }
    bool result = repeatCOM<uint8_t, float, uint8_t>(&Dynamixel2Arduino::setGoalPosition, id, position, 0U);
    if (result)
    {
        lastSetGoalPosition = position;
        errorStatus = false;
    }
    else
    {
        errorStatus = true;
    }
    debugSerial.printf("DXLMotor::setGoalPosition:: id[%d] -> [%f] = %d (record updated: %f)\n", id, position, result, lastSetGoalPosition);
    return result;
}

void DXLMotor::setLED(bool setOn)
{
    if (setOn)
    {
        dxl.ledOn(id);
    }
    else
    {
        dxl.ledOff(id);
    }
}

void DXLMotor::flashLED(uint8_t times, unsigned long interval)
{
    for (uint8_t i = 0; i < times; i++)
    {
        setLED(true);
        delay(interval);
        setLED(false);
        delay(interval);
    }
}

bool DXLMotor::setVelocityLimit(int32_t val)
{
    bool result = repeatCOM<uint8_t, uint8_t, int32_t, uint32_t>(&Dynamixel2Arduino::writeControlTableItem, VELOCITY_LIMIT, id, val, 100);
    debugSerial.printf("DXLMotor::setVelocityLimit:: id[%d] = %d\n", id, result);
    return result;
}

void DXLMotor::resetOPRelatedParamRecords()
{
    lastSetAccelerationProfile = -999;
    lastSetAccelerationProfile = -999;
    lastSetGoalCurrent = -999;
    lastSetGoalPosition = -999;
    lastSetGoalVelocity = -999;
}

bool DXLMotor::repeatCOM(bool (Dynamixel2Arduino::*dxlFunc)())
{
    int count = 0;
    bool result = ((dxl).*(dxlFunc))();
    while (!result)
    {
        result = ((dxl).*(dxlFunc))();
        if (count >= MAX_DXL_PROTOCOL_ATTEMPTS)
        {
            break;
        }
    }
    return result;
}

template <typename T>
bool DXLMotor::repeatCOM(bool (Dynamixel2Arduino::*dxlFunc)(T), T arg)
{
    int count = 0;
    bool result = ((dxl).*(dxlFunc))(arg);
    while (!result && count < MAX_DXL_PROTOCOL_ATTEMPTS)
    {
        result = ((dxl).*(dxlFunc))(arg);
        count++;
    }
    return result;
}

template <typename T1, typename T2>
bool DXLMotor::repeatCOM(bool (Dynamixel2Arduino::*dxlFunc)(T1, T2), T1 arg, T2 arg1)
{
    int count = 0;
    bool result = ((dxl).*(dxlFunc))(arg, arg1);
    while (!result && count < MAX_DXL_PROTOCOL_ATTEMPTS)
    {
        result = ((dxl).*(dxlFunc))(arg, arg1);
        count++;
    }
    return result;
}

template <typename T1, typename T2, typename T3>
bool DXLMotor::repeatCOM(bool (Dynamixel2Arduino::*dxlFunc)(T1, T2, T3), T1 arg, T2 arg1, T3 arg2)
{
    int count = 0;
    bool result = ((dxl).*(dxlFunc))(arg, arg1, arg2);
    while (!result && count < MAX_DXL_PROTOCOL_ATTEMPTS)
    {
        result = ((dxl).*(dxlFunc))(arg, arg1, arg2);
        count++;
    }
    return result;
}

template <typename T1, typename T2, typename T3, typename T4>
bool DXLMotor::repeatCOM(bool (Dynamixel2Arduino::*dxlFunc)(T1, T2, T3, T4), T1 arg, T2 arg1, T3 arg2, T4 arg3)
{
    int count = 0;
    bool result = ((dxl).*(dxlFunc))(arg, arg1, arg2, arg3);
    while (!result)
    {
        result = ((dxl).*(dxlFunc))(arg, arg1, arg2, arg3);
        count++;
    }
    return result;
}

template <typename T1, typename T2, typename T3>
int32_t DXLMotor::repeatCOM(int32_t (Dynamixel2Arduino::*dxlFunc)(T1, T2, T3), T1 arg, T2 arg1, T3 arg2)
{
    int count = 0;
    int32_t result = ((dxl).*(dxlFunc))(arg, arg1, arg2);
    while (result == 0 && count < MAX_DXL_PROTOCOL_ATTEMPTS)
    {
        result = ((dxl).*(dxlFunc))(arg, arg1, arg2);
        count++;
    }
    return result;
}