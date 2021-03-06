#ifndef PIN_TABLE_H
#define PIN_TABLE_H

#include <pins_arduino.h>

const int PIN_LIMIT_SWITCH_ON_REAR = 3;
const int PIN_LIMIT_SWITCH_OFF_REAR = 5;
const int PIN_LIMIT_SWITCH_ON_FRONT = 4;
const int PIN_LIMIT_SWITCH_OFF_FRONT = 6;
const int PIN_PE_SENSOR = 8;
const int PIN_HOOK_CMD = A4;
const int PIN_HINGE_CMD = A5;

const int MOTOR_ID_HOOK = 1;
const int MOTOR_ID_HINGE_X = 2;
const int MOTOR_ID_HINGE_Y = 3;
const float DXL_PROTOCOL_VERSION = 1.0;
const unsigned long DXL_BAUD_RATE = 115200;
const int DXL_DIR_PIN = 2;

const int VELOCITY_HOOK_MOTOR_OPEN = 800;
const int VELOCITY_HOOK_MOTOR_CLOSE = 1823;

#endif