#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <WiFiUdp.h>

#include "crsf.h"
#include "mavlink.h"
#include "config.h"

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2  // GPIO2 для большинства ESP32
#endif

// Config
#define QUEUE_SIZE 20
#define PACKET_TIMEOUT_MS 200
#define TELEMETRY_TIMEOUT_MS 3000
#define UDP_DATA_SEND_INTERVAL_MS   100



// UDP setup
WiFiUDP udp;
IPAddress broadcastIP(255, 255, 255, 255);


// ESPNow data
typedef struct {
    uint8_t data[300];
    uint8_t len;
    uint32_t timestamp;
} ESPNowPacket;

TelemetryData_t telemetriesData;



QueueHandle_t packetQueue = NULL;

void IRAM_ATTR OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
void processingTask(void* parameter);

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== ELRS CRSF Telemetry Parser ===");

    // MAC address setup
    UID[0] &= ~0x01;
    WiFi.mode(WIFI_STA);
    if (esp_wifi_set_mac(WIFI_IF_STA, UID) != ESP_OK) {
        Serial.println("Failed to set MAC address!");
    }

    // ESP-NOW init
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(OnDataRecv);
        Serial.println("ESP-NOW: Ready");

        // Add broadcast peer
        uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, broadcastMac, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        if (esp_now_add_peer(&peerInfo) == ESP_OK) {
            Serial.println("ESP-NOW: Broadcast peer added");
        }
    } else {
        Serial.println("ESP-NOW: Init failed!");
        Serial.println("Restarting in 3 seconds...");
        delay(3000);
        ESP.restart();
    }

    Serial.print("My MAC Address: ");
    Serial.println(WiFi.macAddress());

    // The actual MAC checking
    uint8_t actualMAC[6];
    esp_wifi_get_mac(WIFI_IF_STA, actualMAC);
    Serial.print("Actual MAC: ");
    for(int i = 0; i < 6; i++) {
        Serial.printf("%02X", actualMAC[i]);
        if(i < 5) Serial.print(":");
    }
    Serial.println("\n");

    // Make quee for FreeRTOS task
    packetQueue = xQueueCreate(QUEUE_SIZE, sizeof(ESPNowPacket));
    if (packetQueue == NULL) {
        Serial.println("ERROR: Failed to create packet queue!");
        ESP.restart();
    }
    Serial.printf("Queue created with size %d\n", QUEUE_SIZE);

    // Run task on Core 1
    BaseType_t taskResult = xTaskCreatePinnedToCore(
        processingTask,
        "MAVLinkProc",
        16384,
        NULL,
        3,
        NULL,
        1
    );

    if (taskResult != pdPASS) {
        Serial.println("ERROR: Failed to create processing task!");
        ESP.restart();
    }

    // Create WIFI access point
    bool apStarted = WiFi.softAP(ap_ssid, ap_password);

    if (apStarted) {
        Serial.println("WIFI access point is creatred successfull!");
        Serial.print("SSID: ");
        Serial.println(ap_ssid);
        Serial.print("Password: ");
        Serial.println(ap_password);

        // Get AP address
        IPAddress apIP = WiFi.softAPIP();
        Serial.print("IP address AP: ");
        Serial.println(apIP);
        Serial.print("MAC address AP: ");
        Serial.println(WiFi.softAPmacAddress());

        // Show network info
        Serial.println("Connect to this WIFI access point from other gaget");
    } else {
        Serial.println("WIFI AP creating error");
        while(1) delay(1000);
    }

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.println("Waiting for ELRS telemetry...\n");
}

// Callback to read ESPNow
void IRAM_ATTR OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    // Fast data checking
    if (data_len < 11 || data_len > 300 || packetQueue == NULL) return;

    // Sync bytes checking
    if (data[0] != 0x24 || data[1] != 0x58 || data[2] != 0x3C) return;

    ESPNowPacket packet;
    packet.len = data_len;
    packet.timestamp = micros();

    memcpy(packet.data, data, data_len);

    // Add packet to quee
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t result = xQueueSendToBackFromISR(packetQueue, &packet, &xHigherPriorityTaskWoken);

    // Handle Quee overflow
    if (result == errQUEUE_FULL) {
        ESPNowPacket dummy;
        xQueueReceiveFromISR(packetQueue, &dummy, &xHigherPriorityTaskWoken);
        xQueueSendToBackFromISR(packetQueue, &packet, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}



// Data processinf task
void processingTask(void* parameter) {
    ESPNowPacket packet;
    uint32_t sendDataTime = millis();;

    Serial.println("[TASK] Processing task started on Core 1");

    while (1) {
        if (xQueueReceive(packetQueue, &packet, portMAX_DELAY) == pdTRUE) {

            digitalWrite(LED_BUILTIN, HIGH);

            // Check timeout
            if ((micros() - packet.timestamp) / 1000 > PACKET_TIMEOUT_MS) {
                digitalWrite(LED_BUILTIN, LOW);
                continue;
            }

            // Parse CRSF data
            parseCRSFPacket(packet.data, packet.len, &telemetriesData);

            if (millis() >= sendDataTime) {
                uint8_t* ptrMavlinkData;
                uint16_t dataLength;
                // Build MAVLink stream
                if (buildMAVLinkDataStream(&telemetriesData, &ptrMavlinkData, &dataLength)) {
                    // Send MAVLink stream to UDP
                    udp.beginPacket(broadcastIP, UDP_PORT);
                    udp.write(ptrMavlinkData, dataLength);
                    udp.endPacket();
                    sendDataTime = millis() + UDP_DATA_SEND_INTERVAL_MS;
                }
            }
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void printTelemetry(const TelemetryData_t* td) {
    Serial.println("\n══════════ ELRS TELEMETRY ══════════");

    // Верхняя строка: основные показатели
    Serial.printf("📦 Pkts: %lu | ⏱️ Age: %lums\n",
                 td->statistic.packetCount, millis() - td->lastUpdate);

    // Батарея с графиком
    Serial.print("🔋 Battery: ");
    if (td->battery.voltage >= 4.0) Serial.print("🟢 ");
    else if (td->battery.voltage >= 3.7) Serial.print("🟡 ");
    else if (td->battery.voltage >= 3.3) Serial.print("🔴 ");
    else Serial.print("⛔ ");

    Serial.printf("%.2fV (%.1fA) | ", td->battery.voltage, td->battery.current);
    Serial.printf("%d%% | ", td->battery.remaining);
    Serial.printf("Cap: %lumAh\n", td->battery.capacity);

    // Attitude в компактном виде
    Serial.printf("✈️ Att: P%.0f° R%.0f° Y%.0f°\n",
                 td->attitude.pitch * 57.3, td->attitude.roll * 57.3, td->attitude.yaw * 57.3);


    // Режим полета
    if (strlen(td->flightMode.mode) > 0) {
        Serial.printf("📊 Mode: %s\n", td->flightMode);
    }

    // Статистика пакетов (опционально)
    Serial.print("📊 Packets: ");
    int count = 0;
    for (int i = 0; i < 32; i++) {
        if (td->statistic.crsfPackets[i] > 0) {
            if (count++ > 0) Serial.print(", ");
            Serial.printf("0x%02X:%lu", i, td->statistic.crsfPackets[i]);
        }
    }
    if (count == 0) Serial.print("None");
    Serial.println();

    Serial.println("══════════════════════════════════════");
}

void loop() {
#ifdef DEBUG_TO_LOG
    static uint32_t lastDisplay = 0;
    static uint32_t lastTelemetryPrint = 0;
    static uint32_t lastBlink = 0;
    static bool ledState = false;

    // Blink LED while data are reading
    if (millis() - telemetriesData.lastUpdate < 100) {
        if (millis() - lastBlink >= 50) {
            ledState = !ledState;
            digitalWrite(LED_BUILTIN, ledState);
            lastBlink = millis();
        }
    } else {
        if (millis() - lastBlink >= 1000) {
            ledState = !ledState;
            digitalWrite(LED_BUILTIN, ledState);
            lastBlink = millis();
        }
    }

    // Show status data
    if (millis() - lastDisplay >= 1000) {
        unsigned long age = millis() - telemetriesData.lastUpdate;

        Serial.printf("[STATUS] Age:%lums Packets:%lu Queue:%d/%d",
                     age, telemetriesData.statistic.packetCount,
                     uxQueueMessagesWaiting(packetQueue), QUEUE_SIZE);

        if (age > TELEMETRY_TIMEOUT_MS) {
            Serial.println(" (WAITING)");
        } else if (age > 1000) {
            Serial.println(" (SIGNAL LOST)");
        } else {
            Serial.println(" (ACTIVE)");
        }

        lastDisplay = millis();
    }

    // Show full telemetries data
    if (millis() - lastTelemetryPrint >= 5000) {
        if (telemetriesData.statistick.packetCount > 0 && millis() - telemetriesData.lastUpdate < 2000) {
            printTelemetry(&telemetriesData);
        }
        lastTelemetryPrint = millis();
    }
#endif
}