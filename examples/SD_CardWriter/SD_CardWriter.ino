#include <Arduino.h>
#include "DataWriter.h"
#define DEBUG_SERIAL Serial

typedef struct Datastore
{
  uint32_t timestamp;
  float servo_roll_current;
  float servo_roll_position;
  float servo_roll_velocity;
  float servo_pitch_current;
  float servo_pitch_position;
  float servo_pitch_velocity;
  uint8_t hingeStatus;
  uint8_t hookStatus;
  bool engaged;
} lhm_datastore_t;

String compName = "LHM";
DataWriter logWriter(compName);
lhm_datastore_t datastore;

void setup()
{
  DEBUG_SERIAL.begin(9600);
  while (!DEBUG_SERIAL)
    ;
  logWriter.setup();
  logWriter.enabled = true;
  if (logWriter.isSDCardDetected())
  {
    logWriter.initialise();
  }
}

void loop()
{
  log();
}

void log()
{
  writer_status_t status = logWriter.getStatus();
  
  if (status == WRITER_STATUS_NO_FILE)
  {
    Serial.println("SD card detected, but file is not open.");
    logWriter.initialise();
  }
  else if (status == WRITER_STATUS_NO_CARD)
  {
    logWriter.resetFile();
    Serial.println("No SD card detected.");
    return;
  }
  else if (status == WRITER_STATUS_DISABLED)
  {
    Serial.println("Writer is disabled.");
    return;
  }
  packDatastore();
  logWriter.logData((const uint8_t *)&datastore, sizeof(datastore));
}

inline void packDatastore()
{
  datastore.timestamp = micros();
  datastore.servo_roll_velocity = 1;
  datastore.servo_roll_current = 2;
  datastore.servo_roll_position = 3;
  datastore.servo_pitch_velocity = 0;
  datastore.servo_pitch_current = 1;
  datastore.servo_pitch_position = 3;
  datastore.hingeStatus = 4U;
  datastore.hookStatus = 4U;
  datastore.engaged = 0;
}

// #define SD_CS_PIN 2
// #define TEST_FILE_NAME "testfile.txt"
// // 1024 block file
// #define FILE_SIZE 1024UL*512UL
// #include <SPI.h>

// // use next line to test SD.h
// #include <SD.h>

// // use next two lines to test SdFat
// // #include "SdFat.h"
// // SdFat SD;

// uint8_t buf[512];
// File myFile;

// void setup() {
//   // Open serial communications and wait for port to open:
//   Serial.begin(9600);

//   // Wait for USB Serial
//   while (!Serial) {

//   }
//   if (!SD.begin(SD_CS_PIN)) {
//     Serial.println("SD.begin failed!");
//     return;
//   }
//   for (size_t n = 1; n <= 512; n *= 2) {
//     SD.remove(TEST_FILE_NAME);
//     myFile = SD.open(TEST_FILE_NAME, FILE_WRITE);
//     if (!myFile) {
//       Serial.println("open failed");
//       return;
//     }

//     // Write file data
//     uint32_t us = micros();
//     for (uint32_t i = 0; i < FILE_SIZE; i += n) {
//       if (n != myFile.write(buf, n)) {
//         Serial.println("Write failed");
//         return;
//       }
//     }
//     us = micros() - us;
//     myFile.close();

//     Serial.print("buffer size (bytes): ");
//     Serial.print(n);
//     Serial.print(", time (sec): ");
//     Serial.print(0.000001*us);
//     Serial.print(", rate (KB/sec): ");
//     Serial.println(FILE_SIZE / (0.001 * us));
//   }
// }

// // buffer size (bytes): 1, time (sec): 22.21, rate (KB/sec): 23.61
// // buffer size (bytes): 2, time (sec): 12.78, rate (KB/sec): 41.01
// // buffer size (bytes): 4, time (sec): 8.21, rate (KB/sec): 63.82
// // buffer size (bytes): 8, time (sec): 5.79, rate (KB/sec): 90.60
// // buffer size (bytes): 16, time (sec): 4.61, rate (KB/sec): 113.76
// // buffer size (bytes): 32, time (sec): 4.05, rate (KB/sec): 129.52
// // buffer size (bytes): 64, time (sec): 3.73, rate (KB/sec): 140.38
// // buffer size (bytes): 128, time (sec): 3.61, rate (KB/sec): 145.22
// // buffer size (bytes): 256, time (sec): 3.52, rate (KB/sec): 149.12
// // buffer size (bytes): 512, time (sec): 3.08, rate (KB/sec): 170.24

// void loop() {
//   // nothing happens after setup
// }