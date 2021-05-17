#include <SPI.h>
#include <SD.h>

typedef struct DRMDataStore
{
    uint32_t timestamp;
    float current;
    float position;
    float velocity;
    float d1;
    float d2;
    float d3;
    uint8_t d4;
    bool d5;
} drm_datastore_t;

#define PIN_CHIP_SELECT 2
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
            String fileName = getValidFileName();
            Serial.print(fileName);
            dataFile = SD.open(fileName, O_CREAT | O_WRITE);
            if (dataFile)
            {
                Serial.println(" created. Ready to write data.");
            }
            else
            {
                Serial.println(" cannot be created.");
            }
        }
    }

    inline void flush()
    {
        dataFile.flush();
    }

    inline void close()
    {
        dataFile.close();
    }

    void logData(drm_datastore_t &datastore)
    {
        dataFile.write((const uint8_t *)&datastore, sizeof(datastore));
        dataFile.flush();
        // Serial.println("New data!");
    }

    void logDataDelayedFlush(drm_datastore_t &datastore)
    {
        dataFile.write((const uint8_t *)&datastore, sizeof(datastore));
        datastoreBufPointer++;
        if (datastoreBufPointer == datastore_BUF_SIZE)
        {
            dataFile.flush();
            datastoreBufPointer = 0;
        }
        // Serial.println("New data!");
    }
    
    void logDataBuffed(drm_datastore_t &datastore)
    {
        if (datastoreBufPointer == datastore_BUF_SIZE)
        {
            dataFile.write((const uint8_t *)&datastoreBuf, sizeof(datastoreBuf));
            dataFile.flush();
            datastoreBufPointer = 0;
        }
        datastoreBuf[datastoreBufPointer] = datastore;
        datastoreBufPointer++;
        // Serial.print("New data -> ");
        // Serial.println(datastoreBufPointer);
    }

private:
    File dataFile;
    drm_datastore_t datastoreBuf[datastore_BUF_SIZE];
    uint8_t datastoreBufPointer = 0;

    String getValidFileName()
    {
        uint32_t logCount = 0;
        String fileName;
        while (true)
        {
            fileName = (String)"ds_" + String(logCount) + (String)".bin";
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