#ifndef LINK_HOOK_MODULE_CONTROLLER_H
#define LINK_HOOK_MODULE_CONTROLLER_H

#include <Arduino.h>
#include <USBSerial.h>
#include <Dynamixel2Arduino.h>
#include "LinkHookModuleDataTable.h"
#include "LinkHookModuleMessage.h"
#include "DxlMotor.h"

enum class HookStatus
{
    UNKNOWN = 0,
    OFFLINE = 14,
    ERROR = 29,
    FULLY_OPEN = 43,
    FULLY_CLOSED = 57,
    LOOSE = 71,
    CLOSING = 86,
    OPENNING = 100
};

enum class HingeStatus
{
    UNKNOWN = 0,
    OFFLINE = 14,
    ERROR = 29,
    TORQUE_OFF = 43,
    TAKEOFF_MODE  = 57,
    LANDING_POSITION_IN_TRANSITION = 71,
    LANDING_POSITION_READY = 86,
    SWING_REDUCTION= 100
};

enum class OnOffStatus
{
    OFF,
    ON
};

enum class LimitSwitchStatus
{
    CLOSED,
    OPEN,
    OFFLINE
};

class LHMController
{
public:
    LHMController(HardwareSerial &servoSerial, USBSerial &debugSerial);
    void initiateDxl();
    HookStatus getHookStatus();
    HingeStatus getHingeStatus();
    bool isEngaged();
    bool setSwingReductionMode();
    bool setTakeoffMode();
    bool setLandingPosition();
    bool isAtLandingPosition();
    bool stopHingeMotor();
    void openHook();
    void closeHook();
    void stopHookMotor();
    LimitSwitchStatus getTopLimitSwitchStatus();
    LimitSwitchStatus getBotLimitSwitchStatus();

    DXLMotor hookMotor;
    DXLMotor hingeMotorX;
    DXLMotor hingeMotorY;

private:
    Dynamixel2Arduino dxl;
    HookStatus hookMotionStatus;
    HingeStatus hingeStatus;
    LHMMessage lhmMessage;
    LimitSwitchStatus getLimitSwitchStatus(int on_pin, int off_pin);
    OnOffStatus getPESensorStatus();
    int digitalReadExt(int pin);
};

#endif