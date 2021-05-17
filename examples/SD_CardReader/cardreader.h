#include <Arduino.h>
#include <SD.h>

typedef struct DRMDataStore
{
    uint32_t timestamp;
    float current;
    float position;
    float velocity;
} drm_datastore_t;

#define PIN_CHIP_SELECT 2
#define datastore_BUF_SIZE 32

class CardReader
{
public:
    CardReader() {}
    void start()
    {
        ListDir();
        OpenFile();
    }

private:
    File dataFile;
    Sd2Card card;
    SdVolume volume;
    SdFile root;
    drm_datastore_t dataline;
    String dataFileName;

    void ListDir()
    {
        if (!card.init(SPI_HALF_SPEED, PIN_CHIP_SELECT)) {
            Serial.println("SD.begin failed.");
            return;
        }
        if (!volume.init(card))
        {
            Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
            return;
        }
        root.openRoot(volume);
        Serial.println("\nFiles found on the card (name, date and size in bytes): ");
        root.ls(LS_R | LS_DATE | LS_SIZE);
    }

    void OpenFile()
    {
        if (SD.begin(PIN_CHIP_SELECT))
        {
        }
        else
        {
            Serial.println("Critical Failure. Try again.");
        }
        while (true)
        {
            Serial.println("\nSelect file to read.");
            while (Serial.available() <= 0)
            {
                delay(50);
            }
            String dataFileName = Serial.readString();
            dataFileName.trim();
            Serial.print(dataFileName);
            if (!SD.exists(dataFileName))
            {
                Serial.println(" does not exist.");
                continue;
            }
            Serial.println(" found.");
            dataFile = SD.open(dataFileName, O_READ);
            if (!dataFile)
            {
                Serial.println("Fail to open file. Please select another file.");
                continue;
            }
            else
            {
                Decode();
            }
        }
    }

    void ReadAndPrint()
    {
        Serial.println("timestamp,current,position,velocity");
        while (dataFile.available())
        {
            dataFile.read((uint8_t *)&dataline, sizeof(dataline));
            Serial.printf("%d,%.6f,%.6f,%.6f\n", dataline.timestamp, dataline.current, dataline.position, dataline.velocity);
        }
    }

    void Decode()
    {
        uint32_t lineCount = 0;
        String decodedFileName = "decoded_" + dataFileName + ".txt";
        Serial.print(decodedFileName);
        File decodeFile = SD.open(decodedFileName, FILE_WRITE);
        if (!decodeFile)
        {
            Serial.print(" cannot be created.");
            return;
        }
        Serial.println(" created.");
        while (dataFile.available())
        {
            dataFile.read((uint8_t *)&dataline, sizeof(dataline));
            decodeFile.print(dataline.timestamp);
            decodeFile.print(",");
            decodeFile.print(dataline.current);
            decodeFile.print(",");
            decodeFile.print(dataline.position);
            decodeFile.print(",");
            decodeFile.println(dataline.velocity);
            lineCount ++;
            if (lineCount%100 == 1)
            {
                Serial.printf("Decoding in progress: Decoded [%d] lines. Data peek: %d,%.6f,%.6f,%.6f\n", lineCount, dataline.timestamp, dataline.current, dataline.position, dataline.velocity);
            }
        }
        Serial.printf("File decoded, %d entries.\n", lineCount);
        decodeFile.close();
    }
};