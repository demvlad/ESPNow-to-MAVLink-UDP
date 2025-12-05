#ifndef _MAVLINK_H_
#define _MAVLINK_H_
#include <Arduino.h>
struct TelemetryData_t;
bool buildMAVLinkDataStream(TelemetryData_t* telemetry, uint8_t** ptrMavlinkData, uint16_t* ptrDataLength);
#endif