#ifndef DXL_MOTOR_H
#define DXL_MOTOR_H

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include "LinkHookModuleDataTable.h"

class DXLMotor
{
public:
    DXLMotor(Dynamixel2Arduino &dxl, int motorId);
    int getId() { return id; };
    float getGoalVelocity() { return goalVelocity; };
    float getGoalCurrent() { return goalCurrent; };
    float getGoalPosition() { return goalPosition; };
    bool isInErrorStatus() { return errorStatus; };
    OperatingMode getLastSetOperatingMode() { return operationMode; };

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

    void setLED(bool setOn);
    void flashLED(int times, unsigned long interval=50);

private:
    Dynamixel2Arduino &dxl;
    int id;
    float goalVelocity;
    float goalCurrent;
    float goalPosition;
    OperatingMode operationMode;
    bool errorStatus;
};

#endif