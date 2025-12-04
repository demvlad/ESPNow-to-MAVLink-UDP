This is a ESP32 program. 

It reads ESPNow messages from ELRS backpack (https://www.expresslrs.org/software/backpack-telemetry/), transform CRSF telemetries frames to MAVLink packets and sends MAVLink data by UDP on port 14550.

The programm creates WIFI access point with name "mavlink" and password "12345678".

The src\config.h file contains main settings.

Set yours ELRS TX UUID, what generates from binding phrase. You can see it on ELRS TX html page.

uint8_t UID[6] = {78, 82, 166, 251, 35, 234}; // change these UID on yours TX 
