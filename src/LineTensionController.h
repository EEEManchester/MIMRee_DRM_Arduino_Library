#ifndef MIMREE_DRM_CONTROLLER_LINE_TENSION_CONTROLLER_H
#define MIMREE_DRM_CONTROLLER_LINE_TENSION_CONTROLLER_H

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#include "OlamDataTable.h"
#include "DxlMotor.h"

enum olam_line_status_t: uint8_t
{
    OLAM_LINE_STATUS_UNKNOWN = 0,
    OLAM_LINE_STATUS_ERROR,
    OLAM_LINE_LOOSE,
    OLAM_LINE_IN_TENSION,
    OLAM_LINE_IN_TENSION_REVERSED
};

enum olam_tension_control_status_t: uint8_t
{
    OLAM_TC_STATUS_UNKNOWN = 0,
    OLAM_TC_STATUS_ERROR = 1,
    OLAM_TC_STATUS_POWERED_OFF,
    OLAM_TC_STATUS_POSITION_HOLDING
};

class LineTensionController
{
public:
    LineTensionController(DXLMotor &ltcMotor);

    inline uint8_t getCurrentMode() { return _mode; }
    // inline bool isInHomeSequence() { return _isInHomeSequence; }
    inline uint8_t getMotorId() { return ltcMotor.getId(); }
    olam_line_status_t getLineStatus(bool retryOnError = true);
    
    bool goToHomeAndHold();
    bool prepareEngagement();
    bool detension();
    bool holdPosition();
    bool powerOff();
    olam_tension_control_status_t status();

private:
    DXLMotor &ltcMotor;
    uint8_t _mode;
    bool _isInHomeSequence;
    bool _home();
    bool _prepareEngagement();
    bool _detension();
    bool _holdPosition();
    bool runTillLineInTension(int32_t speed);
    bool runTillLineIsLoose(int32_t speed, int8_t dir);
    bool softEmergencyStop(char *message);
    
    inline uint8_t digitalReadExt(uint8_t pin)
    {
        uint8_t state = digitalRead(pin);
        DEBUG_SERIAL.printf("LineTensionController::digitalReadExt::pin[%d]=%d\n", pin, state);
        return state;
    }
};

#endif