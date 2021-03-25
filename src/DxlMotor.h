#ifndef DXL_MOTOR_H
#define DXL_MOTOR_H

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#ifndef DEBUG_SERIAL_CLASS
#define DEBUG_SERIAL_CLASS USBSerial
#endif

class DXLMotor
{
public:
    DXLMotor(Dynamixel2Arduino &dxl, int motorId, DEBUG_SERIAL_CLASS &debugSerial);
    int getId() { return id; };
    float getLastSetGoalVelocity() { return lastSetGoalVelocity; };
    float getLastSetGoalCurrent() { return lastSetGoalCurrent; };
    float getLastSetGoalPosition() { return lastSetGoalPosition; };
    OperatingMode getLastSetOperatingMode() { return lastSetOperatingMode; };
    int getLastSetVelocityProfile() { return lastSetVelocityProfile; }
    int getLastSetAccelerationProfile() { return lastSetAccelerationProfile; }
    bool isInErrorStatus() { return errorStatus; };

    bool isOnline();
    bool reboot();
    bool isTorqueOn();
    float getCurrentPosition();
    float getCurrentVelocity();
    bool isMoving();
    bool isAtPosition(float position, float positionTolerance = -1);
    bool isAtGoalPosition(float positionTolerance = -1);

    bool setOperatingMode(OperatingMode op);
    bool setTorqueOff();
    bool setTorqueOn();
    bool setGoalVelocity(float velocity);
    bool setGoalCurrent(float current);
    bool setGoalPosition(float position);
    bool setVelocityProfile(int val);
    bool setAccelerationProfile(int val);

    void setLED(bool setOn);
    void flashLED(int times, unsigned long interval = 50);

private:
    Dynamixel2Arduino &dxl;
    DEBUG_SERIAL_CLASS &debugSerial;
    int id;
    bool errorStatus;
    bool setGoalVelocityRaw(float velocity);
    bool setGoalCurrentRaw(float current);
    bool setGoalPositionRaw(float position);
    OperatingMode lastSetOperatingMode = UNKNOWN_OP;
    float lastSetGoalVelocity = -999;
    float lastSetGoalCurrent = -999;
    float lastSetGoalPosition = -999;
    int lastSetVelocityProfile = -999;
    int lastSetAccelerationProfile = -999;
    void resetOPRelatedParamRecords();
    const int MAX_DXL_PROTOCOL_ATTEMPTS = 5;
    bool repeatCOM(bool (Dynamixel2Arduino::*dxlFunc)());
    template <typename T>
    bool repeatCOM(bool (Dynamixel2Arduino::*dxlFunc)(T), T arg);
    template <typename T1, typename T2>
    bool repeatCOM(bool (Dynamixel2Arduino::*dxlFunc)(T1, T2), T1 arg, T2 arg1);
    template <typename T1, typename T2, typename T3>
    bool repeatCOM(bool (Dynamixel2Arduino::*dxlFunc)(T1, T2, T3), T1 arg, T2 arg1, T3 arg2);
    template <typename T1, typename T2, typename T3, typename T4>
    bool repeatCOM(bool (Dynamixel2Arduino::*dxlFunc)(T1, T2, T3, T4), T1 arg, T2 arg1, T3 arg2, T4 arg3);
};

#endif