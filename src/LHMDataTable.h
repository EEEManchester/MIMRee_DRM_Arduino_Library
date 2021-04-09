#ifndef MIMREE_DRM_CONTROLLER_LHM_DATATABLE_H
#define MIMREE_DRM_CONTROLLER_LHM_DATATABLE_H
#ifdef LHM_DEBUG
#define LHM_DEBUG_PRINTF(fmt, ...) DEBUG_SERIAL.printf(fmt, ##__VA_ARGS__)
#define LHM_DEBUG_PRINTLN(a) DEBUG_SERIAL.println(a)
#else
#define LHM_DEBUG_PRINTF(fmt, ...)
#define LHM_DEBUG_PRINTLN(a)
#endif

#include "mavlink/mavlink.h"
#include "OpenCM904EXP.h"

//Serial
#define DXL_SERIAL Serial3
#define COM_SERIAL Serial2
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_CLASS USBSerial

//Pin
#define PIN_LIMIT_SWITCH_CLOSED_TOP 13 //blue
#define PIN_LIMIT_SWITCH_OPEN_TOP 12   //brown
#define PIN_LIMIT_SWITCH_CLOSED_BOT 11 //yellow
#define PIN_LIMIT_SWITCH_OPEN_BOT 10   //purple
#define PIN_PE_SENSOR 7                //white
#define PIN_JETTISON_SERVO_PWM 6

//Communication
#define LHM_MAV_SYS_ID 1
#define LHM_MAV_COMP_ID MAV_COMP_ID_USER31
#define LHM_MAV_BAUDRATE 57600
#define GCS_MAV_COMP_ID 100

//Dxl servo firmware settings
#define MOTOR_ID_HOOK 1
#define MOTOR_ID_HINGE_PITCH 2
#define MOTOR_ID_HINGE_ROLL 3
#define DXL_PROTOCOL_VERSION 2.0
#define DXL_BAUD_RATE 1000000

//Dxl servo user configurations
#define VELOCITY_HOOK_MOTOR_OPEN 230
#define VELOCITY_HOOK_MOTOR_CLOSE -230
#define PROFILE_VELOCITY_VAL 18
#define PROFILE_ACCELERATION_VAL 2
#define HINGE_POS_VERTICAL 2048
#define POSITION_TOLERANCE 13
#define MOVING_THRESHOLD_VELOCITY 1
#define MOVING_THRESHOLD_POSITION POSITION_TOLERANCE

//Motion sequence
// {Motion_count, M_1_servo_id, M_1_pos, M_1_ACC, M_2_servo_id, M_2_pos, M_2_ACC, etc...}
const uint16_t MOTION_SEQ_LANDING[] = {3, MOTOR_ID_HINGE_ROLL, HINGE_POS_VERTICAL, 100, MOTOR_ID_HINGE_PITCH, 1024, 100, MOTOR_ID_HINGE_ROLL, 1024, 50};

//OpenCM 9.04 + 485 EXP specific
#define PIN_BUTTON_JETTISON PIN_BUTTON_1

//Jettison
#define JETTISON_SERVO_VALUE_CLOSE 70
#define JETTISON_SERVO_VALUE_OPEN 160

#define LHM_MSG_TYPE_STATUS_REPORT "LHMS"

enum lhm_mav_msg_id_t : uint32_t
{
    LHM_MAV_MSG_ID_NOT_FOR_US = 9999999,
    LHM_MAV_MSG_ID_HEARTBEAT = MAVLINK_MSG_ID_HEARTBEAT,
    LHM_MAV_MSG_ID_ATTITUDE = MAVLINK_MSG_ID_ATTITUDE,
    LHM_MAV_MSG_ID_COMMAND_INT = MAVLINK_MSG_ID_COMMAND_INT,
    LHM_MAV_MSG_ID_COMMAND_ACK = MAVLINK_MSG_ID_COMMAND_ACK,
    LHM_MAV_MSG_ID_DEBUG_VECT = MAVLINK_MSG_ID_DEBUG_VECT
};

enum lhm_command_id_t : uint16_t
{
    LHM_CMD_ID_UNKNOWN = 0,
    LHM_CMD_ID_HOOK_POWER_OFF = 10,
    LHM_CMD_ID_HOOK_CLOSE = 11,
    LHM_CMD_ID_HOOK_OPEN = 12,
    LHM_CMD_ID_HINGE_POWER_OFF = 20,
    LHM_CMD_ID_HINGE_TAKE_OFF = 21,
    LHM_CMD_ID_HINGE_LANDING = 22,
    LHM_CMD_ID_HINGE_SWING_REDUCTION = 23,
    LHM_CMD_ID_RESET_DYNAMIXEL_COM = 91,
    LHM_CMD_ID_JETTISON = 99,
    LHM_CMD_ID_LOCK_LHM = 98
};

enum lhm_command_progress_t : uint8_t
{
    LHM_CMD_PROG_ERROR = 0,
    LHM_CMD_PROG_COMMAND_ACK = 1,
    LHM_CMD_PROG_MISSION_STARTED = 2,
    LHM_CMD_PROG_MISSION_FINISHED = 3,
};

enum lhm_command_result_t : uint8_t
{
    LHM_CMD_RE_FAILED = 0,
    LHM_CMD_RE_SUCCESSFUL = 1
};

enum on_off_t : bool
{
    OFF = false,
    ON = true
};

enum lhm_hook_status_t : uint8_t
{
    LHM_HOOK_STATUS_UNKNOWN = 0,
    LHM_HOOK_STATUS_OFFLINE,
    LHM_HOOK_STATUS_ERROR,
    LHM_HOOK_STATUS_FULLY_CLOSED = 10,
    LHM_HOOK_STATUS_FULLY_OPEN,
    LHM_HOOK_STATUS_LOOSE,
    LHM_HOOK_STATUS_CLOSING = 20,
    LHM_HOOK_STATUS_OPENNING
};

enum lhm_hinge_status_t : uint8_t
{
    LHM_HINGE_STATUS_UNKNOWN = 0,
    LHM_HINGE_STATUS_OFFLINE,
    LHM_HINGE_STATUS_ERROR,
    LHM_HINGE_STATUS_TORQUE_OFF = 10,
    LHM_HINGE_STATUS_TAKEOFF_MODE = 20,
    LHM_HINGE_STATUS_LANDING_POSITION_IN_TRANSITION = 30,
    LHM_HINGE_STATUS_LANDING_POSITION_READY = 31,
    LHM_HINGE_STATUS_SWING_REDUCTION = 40
};

enum lhm_limit_switch_status_t : uint8_t
{
    LHM_LS_STATUS_OFFLINE = 0,
    LHM_LS_STATUS_ERROR,
    LHM_LS_STATUS_CLOSED = 10,
    LHM_LS_STATUS_OPEN,
};
#endif