#include "crsf.h"
#include "common/mavlink.h"

#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID MAV_COMP_ID_AUTOPILOT1

bool buildMAVLinkDataStream(TelemetryData_t* telemetry, uint8_t** ptrMavlinkData, uint16_t* ptrDataLength) {
    mavlink_message_t mavMsg;
    static uint8_t mavBuffer[MAVLINK_MAX_PACKET_LEN * 4];
    uint16_t dataLength = 0;

    if (ptrMavlinkData) {
        *ptrMavlinkData = mavBuffer;
    } else {
        return false;
    }

    mavlink_msg_gps_raw_int_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        // time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        micros(),
        // fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
        telemetry->satellites > 5 ? 3 : 0,
        // lat Latitude in 1E7 degrees
        telemetry->latitude * 1e7,
        // lon Longitude in 1E7 degrees
        telemetry->longitude * 1e7,
        // alt Altitude in 1E3 meters (millimeters) above MSL
        telemetry->altitude * 1000.0f,
        // eph GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
        UINT16_MAX,
        // epv GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
        UINT16_MAX,
        // vel GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
        telemetry->groundSpeed * 100.0f,
        // cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        telemetry->heading * 100.0f,
        // satellites_visible Number of satellites visible. If unknown, set to 255
        telemetry->satellites,
        // Altitude [mm] (above WGS84, EGM96 ellipsoid). Positive for up.
        telemetry->altitude * 1000.0f,
        // h_acc [mm] Position uncertainty
        UINT32_MAX,
        // v_acc [mm] Altitude uncertainty
        UINT32_MAX,
        // vel_acc [mm/s] Speed uncertainty
        UINT32_MAX,
        // [degE5] Heading / track uncertainty - Unused
        UINT32_MAX,
        //Yaw in earth frame from north. Use 0 if this GPS does not provide yaw - Unused
        0);
    dataLength += mavlink_msg_to_send_buffer(mavBuffer + dataLength, &mavMsg);

    mavlink_msg_attitude_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        // time_boot_ms Timestamp (milliseconds since system boot)
        millis(),
        // roll Roll angle (rad)
        telemetry->roll,
        // pitch Pitch angle (rad)
        telemetry->pitch,
        // yaw Yaw angle (rad)
        telemetry->yaw,
        // rollspeed Roll angular speed (rad/s)
        0,
        // pitchspeed Pitch angular speed (rad/s)
        0,
        // yawspeed Yaw angular speed (rad/s)
        0);
    dataLength += mavlink_msg_to_send_buffer(mavBuffer + dataLength, &mavMsg);

    mavlink_msg_heartbeat_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        // type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        MAV_TYPE_QUADROTOR,
        // autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        MAV_AUTOPILOT_GENERIC,
        // base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
        0,
        // custom_mode A bitfield for use for autopilot-specific flags.
        0,
        // system_status System status flag, see MAV_STATE ENUM
        MAV_STATE_ACTIVE);
    dataLength += mavlink_msg_to_send_buffer(mavBuffer + dataLength, &mavMsg);

    mavlink_msg_sys_status_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        // onboard_control_sensors_present Bitmask showing which onboard controllers and sensors are present.
        //Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure,
        // 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position,
        // 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization,
        // 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
        35843,
        // onboard_control_sensors_enabled Bitmask showing which onboard controllers and sensors are enabled
        35843,
        // onboard_control_sensors_health Bitmask showing which onboard controllers and sensors are operational or have an error.
        35843 & 1023,
        // load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
        0,
        // voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
        telemetry->voltage * 1e3,
        // current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
        telemetry->current * 10,
        // battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
        telemetry->batteryRemaining,
        // drop_rate_comm Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        0,
        // errors_comm Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        0,
        // errors_count1 Autopilot-specific errors
        0,
        // errors_count2 Autopilot-specific errors
        0,
        // errors_count3 Autopilot-specific errors
        0,
        // errors_count4 Autopilot-specific errors
        0,
        // extended parameters, set to zero
        0,
        0,
        0);
    dataLength += mavlink_msg_to_send_buffer(mavBuffer + dataLength, &mavMsg);
/*
    uint16_t voltages[MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN];
    uint16_t voltagesExt[MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_LEN];
    memset(voltages, 0xff, sizeof(voltages));
    memset(voltagesExt, 0, sizeof(voltagesExt));
    voltages[0] = telemetry->voltage * 1e3;

    int16_t batteryAmperage = telemetry->current * 10;
    int32_t amperageConsumed = telemetry->capacity;
    int8_t batteryRemaining = telemetry->batteryRemaining;

    // Temperature: INT16_MAX if unknown
    int16_t temperature = INT16_MAX;

    mavlink_msg_battery_status_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &mavMsg,
        0,                    // id: Battery ID (0 = main battery)
        0,                    // battery_function: 0 = MAV_BATTERY_FUNCTION_UNKNOWN
        0,                    // type: 0 = MAV_BATTERY_TYPE_UNKNOWN (could use MAV_BATTERY_TYPE_LIPO = 1)
        temperature,          // temperature: INT16_MAX = unknown
        voltages,             // voltages[10]: Cell voltages in mV
        batteryAmperage,      // current_battery: Current in cA
        amperageConsumed,     // current_consumed: mAh drawn (CRITICAL for "Capa")
        -1,                   // energy_consumed: -1 = not available (could calculate from Wh if needed)
        batteryRemaining,     // battery_remaining: Percentage 0-100
        0,                    // time_remaining: 0 = not calculated
        MAV_BATTERY_CHARGE_STATE_UNDEFINED, // charge_state: 0 = MAV_BATTERY_CHARGE_STATE_UNDEFINED
        voltagesExt,          // voltages_ext[4]: Cells 11-14, not used
        0,                    // mode: 0 = normal
        0                     // fault_bitmask: 0 = no faults
    );
    dataLength += mavlink_msg_to_send_buffer(mavBuffer + dataLength, &mavMsg);
 */
    if (ptrDataLength) {
        *ptrDataLength = dataLength;
    }

    return dataLength > 0;
}