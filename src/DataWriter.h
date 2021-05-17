#include <SPI.h>
#include <SD.h>
#include <Arduino.h>

#define PIN_CHIP_SELECT 2
#define PIN_CARD_DETECTION 0

enum writer_status_t : uint8_t
{
    WRITER_STATUS_DISABLED = 0,
    WRITER_STATUS_NO_CARD,
    WRITER_STATUS_NO_FILE,
    WRITER_STATUS_OK
};

class DataWriter
{
public:
    DataWriter(const String &compName) : compName(compName) {};

    bool enabled = false;
    uint16_t tempDataSize = 32;

    void setup();
    bool initialise();
    void logData(const uint8_t *buf, size_t bufsize);

    inline void flush()
    {
        tempDataCount = 0;
        dataFile.flush();
    }

    inline void close()
    {
        dataFile.close();
        resetFile();
    }

    inline bool isSDCardDetected()
    {
        if (!useCardDetection)
        {
            return true;
        }
        return digitalRead(PIN_CARD_DETECTION) == LOW;
    }

    inline writer_status_t getStatus()
    {
        if (!enabled)
            return WRITER_STATUS_DISABLED;
        if (!isSDCardDetected())
            return WRITER_STATUS_NO_CARD;
        if (!dataFile)
            return WRITER_STATUS_NO_FILE;
        return WRITER_STATUS_OK;
    }

    inline bool canWrite()
    {
        return getStatus() == WRITER_STATUS_OK;
    }

    inline void resetFile()
    {
        if (!isFileInitialised)
            return;
        dataFile = File();
        tempDataCount = 0;
        isFileInitialised = false;
    }

private:
    String compName;
    String dirName;
    File dataFile;
    uint16_t tempDataCount = 0;
    bool useCardDetection = false;
    bool isFileInitialised = true;
    bool isDirSet = false;

    String getValidFileName();
    bool beginSD();
    bool setupNewDir();
    bool setupNewFile();
};