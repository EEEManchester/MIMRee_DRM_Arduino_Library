//TODO Retry on execution failure

#ifndef MIMREE_DRM_CONTROLLER_LHM_CONTROLLER_H
#define MIMREE_DRM_CONTROLLER_LHM_CONTROLLER_H

#include <Arduino.h>
#include <USBSerial.h>
#include <Dynamixel2Arduino.h>
#include <Servo.h>

#include "LHMDataTable.h"
#include "LHMMessage.h"
#include "DxlMotor.h"
#include "MotionSequence.h"
#include "utilities/LongShortPressButton.h"

class LHMController
{
public:
    LHMController();
    void initiate();
    HookStatus getHookStatus();
    HookStatus getHookMotionStatus() { return hookMotionStatus; }
    HingeStatus getHingeStatus();
    bool isEngaged();
    bool setSwingReductionMode();
    bool setTakeoffMode();
    bool setLandingPosition();
    bool isAtLandingPosition();
    MotionSequenceStatusType getMotionSequenceStatus();
    int8_t nextMotionSequence();
    bool stopHingeMotor();
    bool openHook();
    bool closeHook();
    bool stopHookMotor();
    bool jettison();
    bool lockLHM();
    LimitSwitchStatus getTopLimitSwitchStatus();
    LimitSwitchStatus getBotLimitSwitchStatus();

    DXLMotor hookMotor;
    DXLMotor hingeMotorPitch;
    DXLMotor hingeMotorRoll;
    LHMMessage lhmMessage;
    LongShortPressButton btnJet;

private:
    Dynamixel2Arduino dxl;
    Servo jettisonServo;
    HookStatus hookMotionStatus;
    HingeStatus hingeStatus;
    bool _isInMotionSequence;
    bool _motionSequenceStage;
    LimitSwitchStatus getLimitSwitchStatus(uint8_t on_pin, uint8_t off_pin, bool offlineRetry=true);
    OnOffStatus getPESensorStatus();
    uint8_t digitalReadExt(uint8_t pin);
    MotionSequence currentMotionSequence;
    DXLMotor *motors[3];
};

#endif