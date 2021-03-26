#ifndef OLAM_DATA_TABLE_H
#define OLAM_DATA_TABLE_H

#define DXL_SERIAL Serial3
#define COM_SERIAL Serial
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_CLASS USBSerial

//Pin
const uint8_t PIN_DXL_DIR = 22;
const uint8_t TC_PIN_LIMIT_SWITCH_LOW_NO = 11;
const uint8_t TC_PIN_LIMIT_SWITCH_LOW_NC = 12;
const uint8_t TC_PIN_LIMIT_SWITCH_HIGH_NO = 13;
const uint8_t TC_PIN_LIMIT_SWITCH_HIGH_NC = 14;

const uint16_t TC_SERVO_ENCODER_BIT = 4096;

const uint16_t TC_VELOCITY_MAX = 1023;
const uint16_t TC_VELOCITY_INC = 30;
const uint16_t TC_VELOCITY_INITIAL = 50;
const uint8_t TC_LOOSE_LINE_ROT_COUNT = 8;
const int8_t TC_TENSIONING_ROT_DIR = 1;

//Dxl servo firmware settings
const uint8_t MOTOR_ID_LTC = 1;
const uint8_t MOTOR_ID_AC = 2;
const float DXL_PROTOCOL_VERSION = 2.0;
const unsigned long DXL_BAUD_RATE = 57600;
#endif