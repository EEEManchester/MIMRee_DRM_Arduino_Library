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
    DXLMotor(Dynamixel2Arduino &dxl, uint8_t motorId, DEBUG_SERIAL_CLASS &debugSerial);
    uint8_t getId() { return id; };
    float getLastSetGoalVelocity() { return lastSetGoalVelocity; };
    float getLastSetGoalCurrent() { return lastSetGoalCurrent; };
    float getLastSetGoalPosition() { return lastSetGoalPosition; };
    OperatingMode getLastSetOperatingMode() { return lastSetOperatingMode; };
    int32_t getLastSetVelocityProfile() { return lastSetVelocityProfile; }
    int32_t getLastSetAccelerationProfile() { return lastSetAccelerationProfile; }
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
    bool setVelocityProfile(int32_t val);
    bool setAccelerationProfile(int32_t val);

    void setLED(bool setOn);
    void flashLED(uint8_t times, unsigned long interval = 50);

private:
    const uint8_t MAX_DXL_PROTOCOL_ATTEMPTS = 5;

    Dynamixel2Arduino &dxl;
    DEBUG_SERIAL_CLASS &debugSerial;
    uint8_t id;
    bool errorStatus;
    
    OperatingMode lastSetOperatingMode = UNKNOWN_OP;
    float lastSetGoalVelocity = -32768;
    float lastSetGoalCurrent = -32768;
    float lastSetGoalPosition = -32768;
    int32_t lastSetVelocityProfile = -32768;
    int32_t lastSetAccelerationProfile = -32768;
    void resetOPRelatedParamRecords();

    bool setGoalVelocityRaw(float velocity);
    bool setGoalCurrentRaw(float current);
    bool setGoalPositionRaw(float position);
    
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