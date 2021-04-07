#ifndef MIMREE_DRM_CONTROLLER_LHM_DATATABLE_H
#define MIMREE_DRM_CONTROLLER_LHM_DATATABLE_H


#define DXL_SERIAL Serial3
#define COM_SERIAL Serial2
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_CLASS USBSerial

#ifdef LHM_DEBUG
#define LHM_DEBUG_PRINTF(fmt, ...) DEBUG_SERIAL.printf(fmt, ##__VA_ARGS__)
#define LHM_DEBUG_PRINTLN(a) DEBUG_SERIAL.println(a)
#else
#define LHM_DEBUG_PRINTF(fmt, ...) 
#define LHM_DEBUG_PRINTLN(a)
#endif

#include "mavlink/mavlink.h"
#include "OpenCM904EXP.h"

//Pin
const uint8_t PIN_LIMIT_SWITCH_CLOSED_TOP = 13;      //blue
const uint8_t PIN_LIMIT_SWITCH_OPEN_TOP = 12;        //brown
const uint8_t PIN_LIMIT_SWITCH_CLOSED_BOT = 11;      //yellow
const uint8_t PIN_LIMIT_SWITCH_OPEN_BOT = 10;        //purple
const uint8_t PIN_PE_SENSOR = 7;                     //white
const uint8_t PIN_JETTISON_SERVO_PWM = 6;

//Communication
const uint8_t LHM_MAV_SYS_ID = 1;
const uint8_t LHM_MAV_COMP_ID = MAV_COMP_ID_USER31;
const unsigned long LHM_MAV_BAUDRATE = 57600;

//Dxl servo firmware settings
const uint8_t MOTOR_ID_HOOK = 1;
const uint8_t MOTOR_ID_HINGE_PITCH = 2;
const uint8_t MOTOR_ID_HINGE_ROLL = 3;
const float DXL_PROTOCOL_VERSION = 2.0;
const unsigned long DXL_BAUD_RATE = 1000000;

//Dxl servo user configurations
const int16_t VELOCITY_HOOK_MOTOR_OPEN = 230;
const int16_t VELOCITY_HOOK_MOTOR_CLOSE = -230;
const int32_t PROFILE_VELOCITY_VAL = 18;
const int32_t PROFILE_ACCELERATION_VAL = 2;
const uint16_t HINGE_POS_VERTICAL = 2048;
const uint16_t POSITION_TOLERANCE = 13;
const float MOVING_THRESHOLD_VELOCITY = 1;
const float MOVING_THRESHOLD_POSITION = POSITION_TOLERANCE;

//Motion sequence
// {Motion_count, M_1_servo_id, M_1_pos, M_1_ACC, M_2_servo_id, M_2_pos, M_2_ACC, etc...}
const uint16_t MOTION_SEQ_LANDING[] = {3, MOTOR_ID_HINGE_ROLL, HINGE_POS_VERTICAL, 100, MOTOR_ID_HINGE_PITCH, 1024, 100, MOTOR_ID_HINGE_ROLL, 1024, 50};

//OpenCM 9.04 + 485 EXP specific
const uint8_t PIN_BUTTON_JETTISON = PIN_BUTTON_1;

//Jettison
const uint8_t JETTISON_SERVO_VALUE_CLOSE = 70;
const uint8_t JETTISON_SERVO_VALUE_OPEN = 160;

const char LHM_MSG_TYPE_UNKNOWN[10] = "LHMU";
const char LHM_MSG_TYPE_CMD_IN[10] = "LHMC";
const char LHM_MSG_TYPE_CMD_FB[10] = "LHMFB";
const char LHM_MSG_TYPE_STATUS_REPORT[10] = "LHMSR";

enum class CommandType {
    ERROR = -1,
    UNKNOWN = 0,

    RESET_DYNAMIXEL_COM = 91,
    JETTISON = 99,
    LOCK_LHM = 98,
    
    HINGE_POWER_OFF = 1,
    HINGE_TAKE_OFF = 2,
    HINGE_LANDING = 3,
    HINGE_SWING_REDUCTION = 4,
    
    HOOK_POWER_OFF = 10,
    HOOK_CLOSE = 11,
    HOOK_OPEN = 12,
};

enum class ExecutionResult {
    Failed = 0,
    Successful = 1
};

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

enum class OnOff
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
#endif