#ifndef OLAM_CONTROLLER_H
#define OLAM_CONTROLLER_H

#include <Dynamixel2Arduino.h>
#include <Arduino.h>
#include "DxlMotor.h"
#include "OlamDataTable.h"
#include "LineTensionController.h"

class OLAMController
{
public:
    OLAMController();
    DXLMotor ltcMotor;
    LineTensionController ltController;
    void initiate();

private:
    Dynamixel2Arduino dxl;
};

#endif