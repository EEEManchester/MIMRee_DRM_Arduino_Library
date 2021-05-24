#ifndef MIMREE_DRM_CONTROLLER_DXLMOTOR_H
#define MIMREE_DRM_CONTROLLER_DXLMOTOR_H

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

// #define DXL_DEBUG

#ifdef DXL_DEBUG
// #ifndef DEBUG_SERIAL
// #define DEBUG_SERIAL Serial
// #endif
#define DXL_DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#define DXL_DEBUG_PRINTLN(a) Serial.println(a)
#else
#define DXL_DEBUG_PRINTF(fmt, ...)
#define DXL_DEBUG_PRINTLN(a)
#endif

class DXLMotor
{
public:
    DXLMotor(Dynamixel2Arduino &dxl, uint8_t motorId);
    Dynamixel2Arduino &dxl;
    inline uint8_t getId() { return id; }
    inline float getLastSetGoalVelocity() { return lastSetGoalVelocity; }
    inline float getLastSetGoalCurrent() { return lastSetGoalCurrent; }
    inline float getLastSetGoalPosition() { return lastSetGoalPosition; }
    inline OperatingMode getLastSetOperatingMode() { return lastSetOperatingMode; }
    inline int32_t getLastSetVelocityProfile() { return lastSetVelocityProfile; }
    inline int32_t getLastSetAccelerationProfile() { return lastSetAccelerationProfile; }
    inline bool isInErrorStatus() { return errorStatus; }
    inline float getCurrentPosition()
    {
#ifndef DXL_DEBUG
        return dxl.getPresentPosition(id);
#endif
        float result = dxl.getPresentPosition(id);
        DXL_DEBUG_PRINTF("DXLMotor::getCurrentPosition:: id[%d] = %f\n", id, result);
        return result;
    }
    inline float getCurrentVelocity()
    {
#ifndef DXL_DEBUG
        return dxl.getPresentVelocity(id);
#endif
        float result = dxl.getPresentVelocity(id);
        dxl.getLastStatusPacketError();
        DXL_DEBUG_PRINTF("DXLMotor::getCurrentVelocity:: id[%d] = %f\n", id, result);
        return result;
    }
    inline float getCurrentCurrent()
    {
#ifndef DXL_DEBUG
        return dxl.getPresentCurrent(id);
#endif
        float result = dxl.getPresentCurrent(id);
        DXL_DEBUG_PRINTF("DXLMotor::getCurrentCurrent:: id[%d] = %f\n", id, result);
        return result;
    }

    inline bool isHardwareError(bool fromLastStatusPacket)
    {
        if (!fromLastStatusPacket)
        {
            dxl.ping();
        }
        uint8_t errBit = dxl.getLastStatusPacketError();
        DXL_DEBUG_PRINTF("DXLMotor::isHardwareError:: ErrorBit=%d", errBit);
        bool result = errBit>>7&1;
        DXL_DEBUG_PRINTF(" -> result=%d\n", result);
        return result;
    }

    bool isOnline();
    bool reboot();
    bool isTorqueOn();
    bool isMoving();
    bool isAtPosition(float position, float positionTolerance = 5);
    inline bool isAtGoalPosition(float positionTolerance = 5) { return isAtPosition(lastSetGoalPosition, positionTolerance); }

    bool setOperatingMode(OperatingMode op);
    bool setTorqueOff();
    bool setTorqueOn();
    bool setGoalVelocity(float velocity);
    bool setGoalCurrent(float current);
    bool setGoalPosition(float position);
    bool setVelocityProfile(int32_t val);
    bool setAccelerationProfile(int32_t val);
    bool setVelocityLimit(int32_t val);

    inline void setLED(bool setOn)
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

    void flashLED(uint8_t times, unsigned long interval = 50);

private:
    const uint8_t MAX_DXL_PROTOCOL_ATTEMPTS = 5;

    uint8_t id;
    bool errorStatus;

    OperatingMode lastSetOperatingMode = UNKNOWN_OP;
    float lastSetGoalVelocity = -32768;
    float lastSetGoalCurrent = -32768;
    float lastSetGoalPosition = -32768;
    int32_t lastSetVelocityProfile = -32768;
    int32_t lastSetAccelerationProfile = -32768;
    inline void resetOPRelatedParamRecords()
    {
        lastSetVelocityProfile = -999;
        lastSetAccelerationProfile = -999;
        lastSetGoalCurrent = -999;
        lastSetGoalPosition = -999;
        lastSetGoalVelocity = -999;
    }

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