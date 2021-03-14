//TODO Retry on execution failure

#ifndef LINK_HOOK_MODULE_CONTROLLER_H
#define LINK_HOOK_MODULE_CONTROLLER_H

#include <Arduino.h>
#include <USBSerial.h>
#include <Dynamixel2Arduino.h>
#include <Servo.h>
#include "LinkHookModuleDataTable.h"
#include "LinkHookModuleMessage.h"
#include "DxlMotor.h"

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
    OFF,
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
    bool stopHingeMotor();
    bool openHook();
    bool closeHook();
    bool stopHookMotor();
    bool jettison();
    bool lockLHM();
    LimitSwitchStatus getTopLimitSwitchStatus();
    LimitSwitchStatus getBotLimitSwitchStatus();

    DXLMotor hookMotor;
    DXLMotor hingeMotorX;
    DXLMotor hingeMotorY;
    LHMMessage lhmMessage;

private:
    Dynamixel2Arduino dxl;
    Servo jettisonServo;
    HookStatus hookMotionStatus;
    HingeStatus hingeStatus;
    LimitSwitchStatus getLimitSwitchStatus(int on_pin, int off_pin);
    OnOffStatus getPESensorStatus();
    int digitalReadExt(int pin);
};

#endif