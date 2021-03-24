#ifndef LINK_HOOK_MODULE_DATA_TABLE_H
#define LINK_HOOK_MODULE_DATA_TABLE_H

#define SERIAL_CLASS USBSerial
#define SERIAL2_CLASS UARTClass
#define SERIAL3_CLASS HardwareSerial

//Pin
const int PIN_LIMIT_SWITCH_CLOSED_TOP = 13;      //blue
const int PIN_LIMIT_SWITCH_OPEN_TOP = 12;        //brown
const int PIN_LIMIT_SWITCH_CLOSED_BOT = 11;      //yellow
const int PIN_LIMIT_SWITCH_OPEN_BOT = 10;        //purple
const int PIN_PE_SENSOR = 7;                     //white
const int PIN_JETTISON_SERVO_PWM = 6;

//Communication
const char SERIAL_PREFIX = '$';
const char SERIAL_MESSAGE_TYPE_INDICATOR_CMD = 'C';
const char SERIAL_MESSAGE_TYPE_INDICATOR_FBK = 'F';
const char SERIAL_MESSAGE_TYPE_INDICATOR_STATUS = 'S';

//Dxl servo firmware settings
const int MOTOR_ID_HOOK = 1;
const int MOTOR_ID_HINGE_Y = 2;
const int MOTOR_ID_HINGE_X = 3;
const float DXL_PROTOCOL_VERSION = 2.0;
const unsigned long DXL_BAUD_RATE = 1000000;

//Dxl servo user configurations
const int VELOCITY_HOOK_MOTOR_OPEN = 230;
const int VELOCITY_HOOK_MOTOR_CLOSE = -230;
const int PROFILE_VELOCITY_VAL = 18;
const int PROFILE_ACCELERATION_VAL = 2;
const float HINGE_X_VAL_LANDING_POISITION = 2048;
const float HINGE_Y_VAL_LANDING_POISITION = 1024;
const float POSITION_TOLERANCE = 13;
const float MOVING_THRESHOLD_VELOCITY = 1;
const float MOVING_THRESHOLD_POSITION = POSITION_TOLERANCE;

//Trajectory
// {M_1_servo_id, M_1_pos, M_1_ACC, M_2_servo_id, M_2_pos, M_2_ACC, etc...}
const float TRAJ_LANDING[] = {0, 2048, 20, 1, 1024, 20, 0, 1024, 20};

//OpenCM 9.04 + 485 EXP specific
const int PIN_DXL_DIR = 22;
const int PIN_LED_0 = 14;
const int PIN_LED_1 = 18;
const int PIN_LED_2 = 19;
const int PIN_LED_3 = 20;
const int PIN_BUTTON_1 = 16;
const int PIN_BUTTON_2 = 17;

//Jettison
const int JETTISON_SERVO_VALUE_CLOSE = 70;
const int JETTISON_SERVO_VALUE_OPEN = 160;
#endif