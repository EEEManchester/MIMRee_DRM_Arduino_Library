#include <Arduino.h>
#include <SdFat.h>

typedef struct DRMDataStore
{
    uint32_t timestamp;
    float current;
    float position;
    float velocity;
} drm_datastore_t;

#define PIN_CHIP_SELECT BOARD_SPI2_NSS_PIN
#define datastore_BUF_SIZE 32

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
        if (datastoreBufPointer == datastore_BUF_SIZE)
        {
            dataFile.write(datastoreBuf, sizeof(datastoreBuf));
            dataFile.flush();
            datastoreBufPointer = 0;
        }
        datastoreBuf[datastoreBufPointer] = datastore;
        datastoreBufPointer++;
        Serial.print("New data -> ");
        Serial.println(datastoreBufPointer);
    }

private:
    SdFat SD;
    File dataFile;
    drm_datastore_t datastoreBuf[datastore_BUF_SIZE];
    uint8_t datastoreBufPointer = 0;

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