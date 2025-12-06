#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

// MAC address ELRS Backpack. Look it at Backpack web page.
uint8_t UID[6] = {78, 82, 166, 251, 35, 234};

// WIFI access point setup
const char* ap_ssid = "mavlink";
const char* ap_password = "12345678";  // It needs 8 symbols

const uint16_t UDP_PORT = 14550; // GCS UDP port

//#define DEBUG_TO_LOG    // set this define to log telemetries data
#endif