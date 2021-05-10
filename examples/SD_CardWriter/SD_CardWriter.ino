#include <Arduino.h>
#include <SdFat.h>
#include "cardwriter2.h"
#define DEBUG_SERIAL Serial

CardWriter CW;
drm_datastore_t datastore;

void setup() {
    DEBUG_SERIAL.begin(9600);
    CW.setup();
}

void loop() {
    packDatastore();
    CW.logData(datastore);
}

inline void packDatastore()
{
    datastore.timestamp = micros();
    datastore.current = 1;
    datastore.position = 2;
    datastore.velocity = 3;
}