#ifndef LINK_HOOK_MODULE_DATA_TABLE_H
#define LINK_HOOK_MODULE_DATA_TABLE_H

const int PIN_LIMIT_SWITCH_CLOSED_TOP = 16;      //blue
const int PIN_LIMIT_SWITCH_OPEN_TOP = 17;        //brown
const int PIN_LIMIT_SWITCH_CLOSED_BOT = 18;      //yellow
const int PIN_LIMIT_SWITCH_OPEN_BOT = 19;        //purple
const int PIN_PE_SENSOR = 20;                    //white
const int PIN_CMD_IN = 2;
const int PIN_CMD_FBK_NAME = 3;
const int PIN_CMD_FBK_RESULT = 4;

//PWM values
const int PWM_MAX_PIN_CMD = 1023;
const int PWM_MAX_PIN_FBK = 255;

//Dxl motor firmware settings
const int MOTOR_ID_HOOK = 1;
const int MOTOR_ID_HINGE_Y = 2;
const int MOTOR_ID_HINGE_X = 3;
const float DXL_PROTOCOL_VERSION = 2.0;
const unsigned long DXL_BAUD_RATE = 115200;

//Dxl motor user configurations
const int VELOCITY_HOOK_MOTOR_OPEN = 800;
const int VELOCITY_HOOK_MOTOR_CLOSE = 1823;
const int PROFILE_VELOCITY_VAL = 18;
const int PROFILE_ACCELERATION_VAL = 2;
const float HINGE_X_VAL_LANDING_POISITION = 3072;
const float MOVING_THRESHOLD_VELOCITY = 1;


//OpenCM 9.04 + 485 EXP specific
const int PIN_DXL_DIR = 22;
const int PIN_LED_0 = 14;
const int PIN_LED_1 = 18;
const int PIN_LED_2 = 19;
const int PIN_LED_3 = 20;
const int PIN_BUTTON_1 = 16;
const int PIN_BUTTON_2 = 17;

#endif