#ifndef OLAM_DATA_TABLE_H
#define OLAM_DATA_TABLE_H

#define DXL_SERIAL Serial3
#define COM_SERIAL Serial2
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_CLASS USBSerial

//Pin
#define TC_PIN_LIMIT_SWITCH_LOW_NO 10
#define TC_PIN_LIMIT_SWITCH_LOW_NC 11
#define TC_PIN_LIMIT_SWITCH_HIGH_NO 12
#define TC_PIN_LIMIT_SWITCH_HIGH_NC 15

#define TC_SERVO_ENCODER_BIT 4096

#define TC_VELOCITY_MAX 1022
#define TC_VELOCITY_INC 30
#define TC_VELOCITY_INITIAL 1022
#define TC_LOOSE_LINE_ROT_COUNT 10
#define TC_TENSIONING_ROT_DIR -1

//Dxl servo firmware settings
#define MOTOR_ID_LTC 1
#define MOTOR_ID_AC 2
#define DXL_PROTOCOL_VERSION 2.0
#define DXL_BAUD_RATE 57600

//Communication
#define OLAM_MAV_SYS_ID 1
#define OLAM_MAV_COMP_ID MAV_COMP_ID_USER32
#define OLAM_MAV_BAUDRATE 57600
#define GCS_MAV_SYS_ID 1
#define GCS_MAV_COMP_ID 233

#define OLAM_MAV_MSG_DEBUG_VECT_NAME_STATUS_REPORT "OLAMS"
#define OLAM_MAV_MSG_ID_NOT_FOR_US 9999999u
#define OLAM_MAV_MSG_BUTTON_CHANGE_PASSCODE 32467892

enum olam_control_command_t: uint8_t
{
    OLAM_TC_CMD_ID_RESET = 72,
    OLAM_TC_CMD_ID_UNKNOWN = 80,
    OLAM_TC_CMD_ID_POWER_OFF = 81,
    OLAM_TC_CMD_ID_PREPARE_FOR_ENGAGEMENT = 82,
    OLAM_TC_CMD_ID_HOME = 83,
    OLAM_TC_CMD_ID_DETENSION = 84,
    OLAM_TC_CMD_ID_HOLD_POSITION = 85,
};

enum olam_command_progress_t : uint8_t
{
    OLAM_CMD_PROG_ERROR = 0,
    OLAM_CMD_PROG_COMMAND_ACK = 1,
    OLAM_CMD_PROG_MISSION_STARTED = 2,
    OLAM_CMD_PROG_MISSION_FINISHED = 3,
};

enum olam_command_result_t : uint8_t
{
    OLAM_CMD_RE_FAILED = 0,
    OLAM_CMD_RE_SUCCESSFUL = 1
};
#endif