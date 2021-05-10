#include <Arduino.h>
#include <SdFat.h>
#include "cardreader.h"
#define DEBUG_SERIAL Serial

CardReader CR;
drm_datastore_t datastore;

void setup() {
    DEBUG_SERIAL.begin(9600);
    CR.start();
}

void loop() {
}