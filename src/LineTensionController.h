#ifndef LINE_TENSION_CONTROLLER_H
#define LINE_TENSION_CONTROLLER_H

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include "OlamDataTable.h"
#include "DxlMotor.h"

enum LineStatusType
{
    LINE_STATUS_ERROR = -2,
    LINE_STATUS_UNKNOWN,
    LINE_LOOSE = 0,
    LINE_IN_TENSION,
    LINE_IN_TENSION_REVERSED
};

enum LineTensionControlCommandType
{
    TC_UNKNOWN = -1,
    TC_POWER_OFF,
    TC_HOLD_POSITION,
    TC_DETENSION,
    TC_HOME,
    TC_PREPARE_FOR_ENGAGEMENT,
};

enum LineTensionControlModeType
{
    TC_ERROR = -1,
    TC_POWERED_OFF,
    TC_POSITION_HOLDING
};

class LineTensionController
{
public:
    LineTensionController(DXLMotor &ltcMotor);
    LineTensionControlModeType mode() { return _mode; }
    bool run(LineTensionControlCommandType command);

    bool isInHomeSequence() { return _isInHomeSequence; }
    bool holdHome();
    bool prepareEngagement();
    bool detension();
    bool holdPosition();
    bool powerOff();
    uint8_t motorId() {return ltcMotor.getId();}
    LineStatusType lineStatus(bool retryOnError=true);

private:
    DXLMotor &ltcMotor;
    LineTensionControlModeType _mode;
    bool _isInHomeSequence;
    bool _home();
    bool _prepareEngagement();
    bool _detension();
    bool _holdPosition();
    bool runTillLineInTension(int32_t speed);
    bool runTillLineIsLoose(int32_t speed, int8_t dir);
    bool softEmergencyStop(char *message);
    uint8_t digitalReadExt(uint8_t pin);
};

#endif