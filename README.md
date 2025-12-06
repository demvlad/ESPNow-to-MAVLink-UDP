This is a ESP32 program. 

It reads ESPNow messages from ELRS backpack (https://www.expresslrs.org/software/backpack-telemetry/), transform CRSF telemetries frames to MAVLink packets and sends MAVLink data by UDP on port 14550.

The programm creates WIFI access point with name "mavlink" and password "12345678".

The src\config.h file contains main settings.

Set yours ELRS TX UUID, what generates from binding phrase. You can see it on ELRS TX web page.

uint8_t UID[6] = {78, 82, 166, 251, 35, 234}; // change this UID on yours TX values.

Turn on ELRS transmitter with backpacs ESPNow enabled. 
Run this programm on ESP32 device.
Turn on drone with CRSF telemetry enabled.
Connect to 'mavlink' WIFI access point, what is created ESP32 device. The password is 12345678.
Receive MAVLink telemetry packets by 14550 UDP port by using MissionPlanner or other GCS program.
