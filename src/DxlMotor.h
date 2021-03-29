#ifndef MIMREE_DRM_CONTROLLER_DXLMOTOR_H
#define MIMREE_DRM_CONTROLLER_DXLMOTOR_H

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#ifndef DEBUG_SERIAL_CLASS
#define DEBUG_SERIAL_CLASS USBSerial
#endif

class DXLMotor
{
public:
    DXLMotor(Dynamixel2Arduino &dxl, uint8_t motorId, DEBUG_SERIAL_CLASS &debugSerial);
    Dynamixel2Arduino &dxl;
    uint8_t getId() { return id; };
    inline float getLastSetGoalVelocity() { return lastSetGoalVelocity; };
    inline float getLastSetGoalCurrent() { return lastSetGoalCurrent; };
    inline float getLastSetGoalPosition() { return lastSetGoalPosition; };
    inline OperatingMode getLastSetOperatingMode() { return lastSetOperatingMode; };
    inline int32_t getLastSetVelocityProfile() { return lastSetVelocityProfile; }
    inline int32_t getLastSetAccelerationProfile() { return lastSetAccelerationProfile; }
    inline bool isInErrorStatus() { return errorStatus; };

    bool isOnline();
    bool reboot();
    bool isTorqueOn();
    float getCurrentPosition();
    float getCurrentVelocity();
    bool isMoving();
    bool isAtPosition(float position, float positionTolerance = 5);
    bool isAtGoalPosition(float positionTolerance = 5);

    bool setOperatingMode(OperatingMode op);
    bool setTorqueOff();
    bool setTorqueOn();
    bool setGoalVelocity(float velocity);
    bool setGoalCurrent(float current);
    bool setGoalPosition(float position);
    bool setVelocityProfile(int32_t val);
    bool setAccelerationProfile(int32_t val);
    bool setVelocityLimit(int32_t val);

    void setLED(bool setOn);
    void flashLED(uint8_t times, unsigned long interval = 50);

private:
    const uint8_t MAX_DXL_PROTOCOL_ATTEMPTS = 5;

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

    template <typename T1, typename T2, typename T3>
    int32_t repeatCOM(int32_t (Dynamixel2Arduino::*dxlFunc)(T1, T2, T3), T1 arg, T2 arg1, T3 arg2);
};

#endif