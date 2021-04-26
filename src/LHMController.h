#ifndef MIMREE_DRM_CONTROLLER_LHM_CONTROLLER_H
#define MIMREE_DRM_CONTROLLER_LHM_CONTROLLER_H

// #define MAV_DEBUG
#define MAV_ENABLE_CMD

#include <Arduino.h>
#include <USBSerial.h>
#include <Dynamixel2Arduino.h>
#include <Servo.h>

#include "OpenCM904EXP.h"
#include "LHMDataTable.h"
#include "DxlMotor.h"
#include "MotionSequence.h"
#include "utilities/LongShortPressButton.h"

class LHMController : public OpenCM904EXP
{
public:
    LHMController();

    DXLMotor hookMotor;
    DXLMotor hingeMotorPitch;
    DXLMotor hingeMotorRoll;
    LongShortPressButton btnJet;

    void setup();
    bool initiate();

    lhm_hook_status_t getHookStatus();
    inline lhm_hook_status_t getHookMotionStatus() { return hookMotionStatus; }
    lhm_hinge_status_t getHingeStatus();
    inline bool isEngaged() { return getPESensorStatus() == ON; }
    bool isAtLandingPosition();
    MotionSequenceStatusType getMotionSequenceStatus();

    bool setSwingReductionMode();
    bool setTakeoffMode();
    bool setLandingPosition();
    int8_t nextMotionSequence();
    bool stopHingeMotor();
    bool openHook();
    bool closeHook();
    bool stopHookMotor();
    bool jettison();
    bool lockLHM();

private:
    Dynamixel2Arduino dxl;
    Servo jettisonServo;
    lhm_hook_status_t hookMotionStatus;
    lhm_hinge_status_t hingeStatus;
    bool _isInMotionSequence;
    bool _motionSequenceStage;
    MotionSequence currentMotionSequence;
    DXLMotor *motors[3];

    lhm_limit_switch_status_t getLimitSwitchStatus(uint8_t on_pin, uint8_t off_pin, bool offlineRetry = true);
    lhm_limit_switch_status_t getTopLimitSwitchStatus();
    lhm_limit_switch_status_t getBotLimitSwitchStatus();
    on_off_t getPESensorStatus();

    inline uint8_t digitalReadExt(uint8_t pin)
    {
        uint8_t state = digitalRead(pin);
        LHM_DEBUG_PRINTF("LHMController::digitalReadExt::pin[%d]=%d\n", pin, state);
        return state;
    }
};

#endif