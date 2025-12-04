#ifndef CRSF_H
#define CRSF_H
#include <Arduino.h>

// Телеметрия
struct TelemetryData_t {
    float voltage;
    float current;
    uint32_t capacity;
    uint8_t batteryRemaining;
    float pitch;
    float roll;
    float yaw;
    char flightMode[17];
    double latitude;
    double longitude;
    float altitude;
    float groundSpeed;
    float heading;
    uint8_t satellites;
    uint32_t packetCount;
    uint32_t crsfPackets[256];
    unsigned long lastUpdate;
};

bool parseCRSFPacket(const uint8_t *data, int len, TelemetryData_t* telemetry);
#endif

