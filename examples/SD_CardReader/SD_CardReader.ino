#include <Arduino.h>
#include "cardreader.h"
#define DEBUG_SERIAL Serial

CardReader CR;
drm_datastore_t datastore;

void setup() {
    DEBUG_SERIAL.begin(9600);
    while (!DEBUG_SERIAL) ;
    CR.start();
}

void loop() {
}