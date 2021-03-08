#ifndef LINK_HOOK_MODULE_DATA_TABLE_H
#define LINK_HOOK_MODULE_DATA_TABLE_H

#define SERIAL_CLASS USBSerial
#define SERIAL2_CLASS UARTClass
#define SERIAL3_CLASS HardwareSerial

//Pin
const int PIN_LIMIT_SWITCH_CLOSED_TOP = 21;      //blue
const int PIN_LIMIT_SWITCH_OPEN_TOP = 22;        //brown
const int PIN_LIMIT_SWITCH_CLOSED_BOT = 23;      //yellow
const int PIN_LIMIT_SWITCH_OPEN_BOT = 24;        //purple
const int PIN_PE_SENSOR = 20;                    //white

//Communication
const char SERIAL_PREFIX = '$';
const char SERIAL_MESSAGE_TYPE_INDICATOR_CMD = 'C';
const char SERIAL_MESSAGE_TYPE_INDICATOR_FBK = 'F';
const char SERIAL_MESSAGE_TYPE_INDICATOR_STATUS = 'S';

//Dxl motor firmware settings
const int MOTOR_ID_HOOK = 1;
const int MOTOR_ID_HINGE_Y = 2;
const int MOTOR_ID_HINGE_X = 3;
const float DXL_PROTOCOL_VERSION = 2.0;
const unsigned long DXL_BAUD_RATE = 1000000;

//Dxl motor user configurations
const int VELOCITY_HOOK_MOTOR_OPEN = 800;
const int VELOCITY_HOOK_MOTOR_CLOSE = 1823;
const int PROFILE_VELOCITY_VAL = 18;
const int PROFILE_ACCELERATION_VAL = 2;
const float HINGE_X_VAL_LANDING_POISITION = 3072;
const float HINGE_Y_VAL_LANDING_POISITION = 2048;
const float MOVING_THRESHOLD_VELOCITY = 1;
const float POSITION_TOLERANCE = 13;

//OpenCM 9.04 + 485 EXP specific
const int PIN_DXL_DIR = 22;
const int PIN_LED_0 = 14;
const int PIN_LED_1 = 18;
const int PIN_LED_2 = 19;
const int PIN_LED_3 = 20;
const int PIN_BUTTON_1 = 16;
const int PIN_BUTTON_2 = 17;

#endif