#ifndef MIMREE_DRM_CONTROLLER_OLAM_CONTROLLER_H
#define MIMREE_DRM_CONTROLLER_OLAM_CONTROLLER_H

#include <Dynamixel2Arduino.h>
#include <Arduino.h>

#include "DxlMotor.h"
#include "OlamDataTable.h"
#include "OpenCM904EXP.h"
#include "LineTensionController.h"
#include "utilities/LongShortPressButton.h"

class OLAMController : public OpenCM904EXP
{
public:
    OLAMController();
    DXLMotor ltcMotor;
    LineTensionController ltController;
    void initiate();
    LongShortPressButton homeButton;
    LongShortPressButton engagementButton;

private:
    Dynamixel2Arduino dxl;
};

#endif