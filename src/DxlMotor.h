#ifndef DXL_MOTOR_H
#define DXL_MOTOR_H

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include "LinkHookModuleDataTable.h"
#include "LinkHookModuleMessage.h"

class DXLMotor
{
public:
    DXLMotor(Dynamixel2Arduino &dxl, int motorId, LHMMessage &lhmMessage);
    int getId() { return id; };
    float getLastSetGoalVelocity() { return goalVelocity; };
    float getLastSetGoalCurrent() { return goalCurrent; };
    float getLastSetGoalPosition() { return goalPosition; };
    OperatingMode getLastSetOperatingMode() { return operationMode; };
    bool isInErrorStatus() { return errorStatus; };

    bool isOnline();
    bool isTorqueOn();
    float getCurrentPosition();
    float getCurrentVelocity();
    bool isMoving();
    bool isAtPosition(float position);
    bool isAtGoalPosition();

    bool setOperatingMode(OperatingMode op);
    bool setTorqueOff();
    bool setTorqueOn();
    bool setGoalVelocity(float velocity);
    bool setGoalCurrent(float current);
    bool setGoalPosition(float position);
    bool setVelocityProfile(int val);
    bool setAccelerationProfile(int val);

    void setLED(bool setOn);
    void flashLED(int times, unsigned long interval=50);

private:
    Dynamixel2Arduino &dxl;
    LHMMessage &lhmMessage;
    int id;
    float goalVelocity;
    float goalCurrent;
    float goalPosition;
    OperatingMode operationMode;
    bool errorStatus;
    bool setGoalVelocityRaw(float velocity);
    bool setGoalCurrentRaw(float current);
    bool setGoalPositionRaw(float position);
};

#endif