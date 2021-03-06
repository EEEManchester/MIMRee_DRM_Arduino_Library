#include <Arduino.h>
#include <SdFat.h>

typedef struct DRMDataStore
{
    uint32_t timestamp;
    float current;
    float position;
    float velocity;
} drm_datastore_t;

#define PIN_CHIP_SELECT 2

class CardWriter
{
public:
    CardWriter() {}
    void setup()
    {
        if (!SD.begin(PIN_CHIP_SELECT))
        {
            Serial.println("SD.begin failed.");
        }
        else
        {
            Serial.println("SD card initialized.");
            dataFile = SD.open(getValidFileName(), O_CREAT | O_WRITE);
        }
    }

    void logData(drm_datastore_t &datastore)
    {
        dataFile.write((const uint8_t *)&datastore, sizeof(datastore));
        dataFile.flush();
        // Serial.println("New data!");
    }

private:
    SdFat SD;
    File dataFile;

    String getValidFileName()
    {
        uint32_t logCount = 0;
        String fileName;
        while (true)
        {
            fileName = (String)"datastore_" + String(logCount) + (String)".bin";
            if (SD.exists(fileName))
            {
                logCount++;
            }
            else
            {
                return fileName;
            }
        }
    }
};