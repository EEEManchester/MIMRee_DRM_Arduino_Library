#ifndef LHM_DATA_TABLE_H
#define LHM_DATA_TABLE_H

#define DXL_SERIAL Serial3
#define COM_SERIAL Serial
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_CLASS USBSerial

//Pin
const uint8_t PIN_LIMIT_SWITCH_CLOSED_TOP = 13;      //blue
const uint8_t PIN_LIMIT_SWITCH_OPEN_TOP = 12;        //brown
const uint8_t PIN_LIMIT_SWITCH_CLOSED_BOT = 11;      //yellow
const uint8_t PIN_LIMIT_SWITCH_OPEN_BOT = 10;        //purple
const uint8_t PIN_PE_SENSOR = 7;                     //white
const uint8_t PIN_JETTISON_SERVO_PWM = 6;

//Communication
const char SERIAL_PREFIX = '$';
const char SERIAL_MESSAGE_TYPE_INDICATOR_CMD = 'C';
const char SERIAL_MESSAGE_TYPE_INDICATOR_FBK = 'F';
const char SERIAL_MESSAGE_TYPE_INDICATOR_STATUS = 'S';

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
const uint8_t PIN_DXL_DIR = 22;
const uint8_t PIN_LED_0 = 14;
const uint8_t PIN_LED_1 = 18;
const uint8_t PIN_LED_2 = 19;
const uint8_t PIN_LED_3 = 20;
const uint8_t PIN_BUTTON_1 = 16;
const uint8_t PIN_BUTTON_2 = 17;

//Jettison
const uint8_t JETTISON_SERVO_VALUE_CLOSE = 70;
const uint8_t JETTISON_SERVO_VALUE_OPEN = 160;
#endif