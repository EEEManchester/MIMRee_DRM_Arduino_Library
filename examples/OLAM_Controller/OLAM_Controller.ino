#include <OlamController.h>
#include <OlamDataTable.h>

const int STATUS_REPORT_INTERVAL = 1000;
const int COMMAND_IN_CHECK_INTERVAL = 0;

OLAMController olam = OLAMController();

long prevStatusReportTime = 0;
long prevCommandInCheckTime = 0;
LineTensionControlCommandType ltcCommand = TC_UNKNOWN;
LineTensionControlCommandType prevltcCommand = TC_UNKNOWN;
LineTensionControlCommandType defaultltcCommand = TC_DETENSION;

void setup()
{
  DEBUG_SERIAL.begin(9600);
  olam.initiate();
  // pulseLEDSequantial();
  DEBUG_SERIAL.println("Initialized.");

  pinMode(16, INPUT);
  pinMode(17, INPUT);
  ltcCommand = defaultltcCommand;
}

void loop()
{
  if ((int)millis() - prevStatusReportTime > STATUS_REPORT_INTERVAL)
  {
    prevStatusReportTime = millis();
    LineStatusType ls = olam.ltController.lineStatus();
    DEBUG_SERIAL.pringf("Line status is %d\n", (int)ls);
  }
  if ((int)millis() - prevCommandInCheckTime > COMMAND_IN_CHECK_INTERVAL)
  {
    prevCommandInCheckTime = millis();
    // checkCommandIn();
  }
  // checkCommandIn();
  // olam.ltController.run(ltcCommand);
}

void checkCommandIn()
{
  if (digitalRead(16) == HIGH)
  {
    if (prevltcCommand == TC_PREPARE_FOR_ENGAGEMENT)
    {
      ltcCommand = defaultltcCommand;
      prevltcCommand = defaultltcCommand;
    }
    if (olam.ltController.run(TC_PREPARE_FOR_ENGAGEMENT))
    {
      ltcCommand = TC_UNKNOWN;
      prevltcCommand = TC_PREPARE_FOR_ENGAGEMENT;
    }
    else
    {
      ltcCommand = defaultltcCommand;
    }
    return;
  }
  if (digitalRead(17) == HIGH)
  {
    if (prevltcCommand == TC_HOME)
    {
      ltcCommand = defaultltcCommand;
      prevltcCommand = defaultltcCommand;
    }
    if (olam.ltController.run(TC_HOME))
    {
      ltcCommand = TC_UNKNOWN;
      prevltcCommand = TC_HOME;
    }
    else
    {
      ltcCommand = defaultltcCommand;
    }
    return;
  }
}