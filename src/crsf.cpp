#include <Arduino.h>
#include "crsf.h"

// CRSF Frame Types
typedef enum {
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_VARIO = 0x07,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_BARO_ALTITUDE = 0x09,
    CRSF_FRAMETYPE_HEARTBEAT = 0x0B,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17,
    CRSF_FRAMETYPE_LINK_STATISTICS_RX = 0x1C,
    CRSF_FRAMETYPE_LINK_STATISTICS_TX = 0x1D,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
    CRSF_FRAMETYPE_COMMAND = 0x32,
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C,
    CRSF_FRAMETYPE_DISPLAYPORT_CMD = 0x7D,
} crsf_frame_type_e;

// CRSF data struct
typedef struct {
    uint16_t voltage;       // mV * 100
    uint16_t current;       // mA * 100
    uint32_t capacity : 24; // mAh
    uint8_t remaining;      // percent
} crsfBatterySensor_t;

typedef struct {
    int16_t pitch;          // rad / 10000
    int16_t roll;           // rad / 10000
    int16_t yaw;            // rad / 10000
} crsfAttitude_t;

typedef struct {
    int32_t latitude;       // degree / 10,000,000
    int32_t longitude;      // degree / 10,000,000
    uint16_t groundspeed;   // km/h * 10
    uint16_t heading;       // degree * 100
    uint16_t altitude;      // meters (meters + 1000)
    uint8_t satellites;
} crsfGps_t;

typedef struct {
    uint8_t uplinkRSSI1;          // RSSI of uplink (signal strength)
    uint8_t uplinkRSSI2;          // RSSI of uplink (signal strength)
    uint8_t uplinkLinkQuality;    // Link quality of uplink (0-100%)
    int8_t uplinkSNR;             // SNR of uplink
    uint8_t activeAntenna;        // Active antenna
    uint8_t rfMode;               // RF mode
    uint8_t uplinkTXPower;        // Transmit power of uplink (enum)
    uint8_t downlinkRSSI;         // RSSI of downlink (signal strength)
    uint8_t downlinkLinkQuality;  // Link quality of downlink (0-100%)
    int8_t downlinkSNR;           // SNR of downlink
} crsfLinkStatistics_t;

// big-endian transform
int16_t bigEndian16(const uint8_t* bytes) {
    return (bytes[0] << 8) | bytes[1];
}


int32_t bigEndian24(const uint8_t* bytes) {
    return (bytes[0] << 16) | (bytes[1] << 8) | bytes[2];
}

int32_t bigEndian32(const uint8_t* bytes) {
    return (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
}

// ExpressLRS CRC checking
uint8_t crsfCRC(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Data parser
bool parseCRSFPacket(const uint8_t *data, int len, TelemetryData_t* telemetry) {
    // Fast checking
    if (len < 11) return false;
    if (data[0] != 0x24 || data[1] != 0x58 || data[2] != 0x3C) return false;

    // CRSF data start point
    const uint8_t *crsfData = data + 8;
    int crsfLen = len - 8;
    if (crsfLen < 4) return false;

    // read header
    uint8_t frame_len = crsfData[1];
    uint8_t frame_type = crsfData[2];

    if (frame_len < 3 || frame_len > crsfLen) return false;

    uint8_t payload_len = frame_len - 2;
    const uint8_t* payload = crsfData + 3;

    // Statistic update
    telemetry->statistic.packetCount++;
    if (frame_type < 256) {
        telemetry->statistic.crsfPackets[frame_type]++;
    }
    telemetry->lastUpdate = millis();

    // Data packets handle
    switch (frame_type) {

        case CRSF_FRAMETYPE_GPS: // GPS
            if (payload_len >= 15) {
                telemetry->gps.enabled = true;
                telemetry->gps.latitude = bigEndian32(payload) / 10000000.0; // [degree]
                telemetry->gps.longitude = bigEndian32(payload + 4) / 10000000.0; // [degree]
                telemetry->gps.groundSpeed = bigEndian16(payload + 8) / 10.0f; // [m/s]
                telemetry->gps.heading = bigEndian16(payload + 10) * 0.01f; // [degree]
                telemetry->gps.altitude = bigEndian16(payload + 12) - 1000.0f; // [m] The BF CRFS telemetry add  +1000
                telemetry->gps.satellites = payload[14];
            }
            break;

        case CRSF_FRAMETYPE_BATTERY_SENSOR: // Battery
            if (payload_len >= 8) {
                telemetry->battery.enabled = true;
                telemetry->battery.voltage = bigEndian16(payload) * 0.1f;
                telemetry->battery.current = bigEndian16(payload + 2) * 0.1f;
                telemetry->battery.capacity = bigEndian24(payload + 4);
                telemetry->battery.remaining = payload[7];
            }
            break;

        case CRSF_FRAMETYPE_ATTITUDE: // Attitude
            if (payload_len >= 6) {
                telemetry->attitude.enabled = true;
                telemetry->attitude.pitch = bigEndian16(payload) / 10000.0f; // rad*10000 → rad
                telemetry->attitude.roll = bigEndian16(payload + 2) / 10000.0f; // rad*10000 → rad
                telemetry->attitude.yaw = bigEndian16(payload + 4) / 10000.0f; // rad*10000 → rad
            }
            break;

        case CRSF_FRAMETYPE_FLIGHT_MODE: // Flight Mode
            if (payload_len >= 1) {
                telemetry->flightMode.enabled = true;
                int len = payload_len < 16 ? payload_len : 16;
                memcpy(telemetry->flightMode.mode, payload, len);
                telemetry->flightMode.mode[len] = '\0';
                telemetry->flightMode.armed = true; // TODO: perse mode string to define arm flag
            }
            break;
    }

    return true;
}
