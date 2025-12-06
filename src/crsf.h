#ifndef CRSF_H
#define CRSF_H
#include <Arduino.h>

// Телеметрия
struct TelemetryData_t {
    struct {
        bool enabled;
        float voltage;
        float current;
        uint32_t capacity;
        uint8_t remaining;
    } battery;
    struct {
        bool enabled;
        float pitch;
        float roll;
        float yaw;
     } attitude;
    struct {
        bool enabled;
        char mode[17];
        bool armed;
    } flightMode;
    struct {
        bool enabled;
        double latitude;
        double longitude;
        float altitude;
        float groundSpeed;
        float heading;
        uint8_t satellites;
    } gps;
    struct {
        uint32_t packetCount;
        uint32_t crsfPackets[256];
    } statistic;
    unsigned long lastUpdate;
};

bool parseCRSFPacket(const uint8_t *data, int len, TelemetryData_t* telemetry);
#endif

