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

class CardReader
{
public:
    CardReader() {}
    void start()
    {
        if (!SD.begin(PIN_CHIP_SELECT))
        {
            Serial.println("SD.begin failed.");
        }
        else
        {
            Serial.println("SD card initialized.");
            OpenFile();
            ReadAndPrint();
        }
    }

private:
    SdFat SD;
    File dataFile;
    drm_datastore_t dataline;

    void OpenFile()
    {
        Serial.println("File list:");
        SD.ls(LS_SIZE);
        Serial.println("Select file to read:");
        while (false)
        {
            while (Serial.available() <= 0)
            {
                delay(50);
            }
            String fname = Serial.readString();
            if (!SD.exists(fname))
            {
                Serial.println("File does not exist.");
                continue;
            }
            dataFile = SD.open(fname, O_READ);
        }
    }

    void ReadAndPrint()
    {
        Serial.println("timestamp,current,position,velocity");
        while (dataFile.available())
        {
            dataFile.read((uint8_t *)&dataline, sizeof(dataline));
            Serial.printf("%d,%.6f,%.6f,%.6f", dataline.timestamp, dataline.current, dataline.position, dataline.velocity);
        }
    }
};