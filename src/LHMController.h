//TODO Retry on execution failure

#ifndef LHM_CONTROLLER_H
#define LHM_CONTROLLER_H

#include <Arduino.h>
#include <USBSerial.h>
#include <Dynamixel2Arduino.h>
#include <Servo.h>
#include "LHMDataTable.h"
#include "LHMMessage.h"
#include "DxlMotor.h"
#include "MotionSequence.h"

enum class HookStatus
{
    UNKNOWN = 0,
    OFFLINE,
    ERROR,
    FULLY_CLOSED = 10,
    FULLY_OPEN,
    LOOSE,
    CLOSING = 20,
    OPENNING
};

enum class HingeStatus
{
    UNKNOWN = 0,
    OFFLINE,
    ERROR,
    TORQUE_OFF = 10,
    TAKEOFF_MODE = 20,
    LANDING_POSITION_IN_TRANSITION = 30,
    LANDING_POSITION_READY = 31,
    SWING_REDUCTION = 40
};

enum class OnOffStatus
{
    OFF = 0,
    ON
};

enum class LimitSwitchStatus
{
    OFFLINE = 0,
    ERROR,
    CLOSED = 10,
    OPEN,
};

class LHMController
{
public:
    LHMController(HardwareSerial &servoSerial, COM_SERIAL_CLASS &comSerial, DEBUG_SERIAL_CLASS &debugSerial);
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

private:
    Dynamixel2Arduino dxl;
    Servo jettisonServo;
    HookStatus hookMotionStatus;
    HingeStatus hingeStatus;
    bool _isInMotionSequence;
    bool _motionSequenceStage;
    LimitSwitchStatus getLimitSwitchStatus(int on_pin, int off_pin);
    OnOffStatus getPESensorStatus();
    int digitalReadExt(int pin);
    MotionSequence *currentMotionSequence;
};

#endif