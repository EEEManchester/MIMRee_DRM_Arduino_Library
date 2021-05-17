#include "DataWriter.h"

void DataWriter::setup()
{
    if (PIN_CARD_DETECTION >= 0)
    {
        pinMode(PIN_CARD_DETECTION, INPUT_PULLUP);
        useCardDetection = true;
    }
}

bool DataWriter::initialise()
{
    if (!beginSD() || !setupNewDir() || !setupNewFile())
    {
        return false;
    }
}

void DataWriter::logData(const uint8_t *buf, size_t bufsize)
{
    dataFile.write(buf, bufsize);
    tempDataCount++;
    if (tempDataCount == tempDataSize)
    {
        flush();
    }
}

bool DataWriter::beginSD()
{
    if (!SD.begin(PIN_CHIP_SELECT))
    {
        Serial.println("SD.begin failed.");
        return false;
    }
    Serial.println("SD card initialized.");
    return true;
}

bool DataWriter::setupNewDir()
{
    if (isDirSet)
    {
        SD.mkdir(dirName);
        return true;
    }
    uint16_t logCount = 0;
    dirName = compName + (String)logCount;
    while (SD.exists(dirName))
    {
        if (logCount > 65532)
        {
            return false;
        }
        logCount++;
        dirName = compName + (String)logCount;
    }
    SD.mkdir(dirName);
    dirName = "/" + dirName + "/";
    isDirSet = true;
    return true;
}

bool DataWriter::setupNewFile()
{
    String fileName = getValidFileName();
    Serial.print(fileName);
    dataFile = SD.open(fileName, O_CREAT | O_WRITE);
    if (dataFile)
    {
        Serial.println(" created. Ready to write data.");
        isFileInitialised = true;
        return true;
    }
    Serial.println(" cannot be created.");
    return false;
}

String DataWriter::getValidFileName()
{
    uint32_t logCount = 1;
    uint32_t time = millis() / 1000;
    uint8_t hours = time / 3600;
    uint8_t minutes = time % 3600 / 60;
    uint8_t seconds = time % 60;
    String prefix = (String)dirName + (String)hours + "_" + (String)minutes + "_" + (String)seconds;
    String ext = ".bin";
    String finalFilename = prefix + ext;
    while (SD.exists(finalFilename))
    {
        finalFilename = prefix + String(logCount) + ext;
        logCount++;
    }
    return finalFilename;
}