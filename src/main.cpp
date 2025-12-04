#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "crsf.h"
#include "mavlink.h"

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2  // GPIO2 для большинства ESP32
#endif

// Конфигурация
#define QUEUE_SIZE 20
#define PACKET_TIMEOUT_MS 200
#define TELEMETRY_TIMEOUT_MS 3000

// MAC адрес как в рабочем коде
uint8_t UID[6] = {78, 82, 166, 251, 35, 234}; // {0x4E, 0x52, 0xA6, 0xFB, 0x23, 0xEA}


// Структура для очереди
typedef struct {
    uint8_t data[300];
    uint8_t len;
    uint32_t timestamp;
} ESPNowPacket;


// Глобальные объекты
QueueHandle_t packetQueue = NULL;
TelemetryData telemetry = {0};

// Прототипы
void IRAM_ATTR OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
void processingTask(void* parameter);

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== ELRS CRSF Telemetry Parser ===");

    // Устанавливаем MAC-адрес
    UID[0] &= ~0x01;
    WiFi.mode(WIFI_STA);
    if (esp_wifi_set_mac(WIFI_IF_STA, UID) != ESP_OK) {
        Serial.println("Failed to set MAC address!");
    }

    // Инициализация ESP-NOW
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(OnDataRecv);
        Serial.println("ESP-NOW: Ready");

        // Добавляем broadcast peer для приема от всех
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

    // Проверяем реальный MAC
    uint8_t actualMAC[6];
    esp_wifi_get_mac(WIFI_IF_STA, actualMAC);
    Serial.print("Actual MAC: ");
    for(int i = 0; i < 6; i++) {
        Serial.printf("%02X", actualMAC[i]);
        if(i < 5) Serial.print(":");
    }
    Serial.println("\n");

    // Инициализация телеметрии
    memset(&telemetry, 0, sizeof(telemetry));
    telemetry.lastUpdate = 0;
    telemetry.currentRSSI = -100;
    telemetry.linkQuality = 0;
    strcpy(telemetry.flightMode, "Unknown");

    // Создание очереди для FreeRTOS задачи
    packetQueue = xQueueCreate(QUEUE_SIZE, sizeof(ESPNowPacket));
    if (packetQueue == NULL) {
        Serial.println("ERROR: Failed to create packet queue!");
        ESP.restart();
    }
    Serial.printf("Queue created with size %d\n", QUEUE_SIZE);

    // Запуск задачи обработки на ядре 1
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

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.println("Waiting for ELRS telemetry...\n");
}

// Callback с правильной сигнатурой
void IRAM_ATTR OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {

    // Быстрая проверка
    if (data_len < 11 || data_len > 300 || packetQueue == NULL) return;

    // Проверка ELRS синхробайтов
    if (data[0] != 0x24 || data[1] != 0x58 || data[2] != 0x3C) return;

    ESPNowPacket packet;
    packet.len = data_len;
    packet.timestamp = micros();

    // Копируем данные
    memcpy(packet.data, data, data_len);

    // Отправляем в очередь из прерывания
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t result = xQueueSendToBackFromISR(packetQueue, &packet, &xHigherPriorityTaskWoken);

    // Обработка переполнения очереди
    if (result == errQUEUE_FULL) {
        ESPNowPacket dummy;
        xQueueReceiveFromISR(packetQueue, &dummy, &xHigherPriorityTaskWoken);
        xQueueSendToBackFromISR(packetQueue, &packet, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}



// Задача обработки
void processingTask(void* parameter) {
    ESPNowPacket packet;

    Serial.println("[TASK] Processing task started on Core 1");

    while (1) {
        if (xQueueReceive(packetQueue, &packet, portMAX_DELAY) == pdTRUE) {

            digitalWrite(LED_BUILTIN, HIGH);

            // Проверка таймаута
            if ((micros() - packet.timestamp) / 1000 > PACKET_TIMEOUT_MS) {
                digitalWrite(LED_BUILTIN, LOW);
                continue;
            }

            // Парсинг
            parseCRSFPacket(packet.data, packet.len, &telemetry));

            if (buildMAVLinkData(&telemetry, uint8_t** ptrMavlinkData, uint16_t* ptrDataLength)) {

            }

            digitalWrite(LED_BUILTIN, LOW);
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void printTelemetry(const TelemetryData* td) {
    Serial.println("\n══════════ ELRS TELEMETRY ══════════");

    // Верхняя строка: основные показатели
    Serial.printf("📡 RSSI: %ddBm | 📦 Pkts: %lu | ⏱️ Age: %lums\n",
                 td->currentRSSI, td->packetCount, millis() - td->lastUpdate);

    // Батарея с графиком
    Serial.print("🔋 Battery: ");
    if (td->voltage >= 4.0) Serial.print("🟢 ");
    else if (td->voltage >= 3.7) Serial.print("🟡 ");
    else if (td->voltage >= 3.3) Serial.print("🔴 ");
    else Serial.print("⛔ ");

    Serial.printf("%.2fV (%.1fA) | ", td->voltage, td->current);
    Serial.printf("%d%% | ", td->batteryRemaining);
    Serial.printf("Cap: %lumAh\n", td->capacity);

    // Attitude в компактном виде
    Serial.printf("✈️ Att: P%.0f° R%.0f° Y%.0f°\n",
                 td->pitch, td->roll, td->yaw);

    // Связь
    Serial.printf("📶 Link: UL %ddBm | DL %ddBm | LQ %d%%\n",
                 td->uplinkRSSI1, td->downlinkRSSI, td->uplinkLinkQuality);

    // Режим полета
    if (strlen(td->flightMode) > 0) {
        Serial.printf("📊 Mode: %s\n", td->flightMode);
    }

    // Статистика пакетов (опционально)
    Serial.print("📊 Packets: ");
    int count = 0;
    for (int i = 0; i < 32; i++) {
        if (td->crsfPackets[i] > 0) {
            if (count++ > 0) Serial.print(", ");
            Serial.printf("0x%02X:%lu", i, td->crsfPackets[i]);
        }
    }
    if (count == 0) Serial.print("None");
    Serial.println();

    Serial.println("══════════════════════════════════════");
}

void loop() {
    static uint32_t lastDisplay = 0;
    static uint32_t lastTelemetryPrint = 0;
    static uint32_t lastBlink = 0;
    static bool ledState = false;

    // Мигаем LED когда активны пакеты
    if (millis() - telemetry.lastUpdate < 100) {
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

    // Вывод статуса каждую секунду
    if (millis() - lastDisplay >= 1000) {
        unsigned long age = millis() - telemetry.lastUpdate;

        Serial.printf("[STATUS] Age:%lums RSSI:%d Packets:%lu Queue:%d/%d",
                     age, telemetry.currentRSSI, telemetry.packetCount,
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

    // Вывод подробной телеметрии каждые 5 секунд
    if (millis() - lastTelemetryPrint >= 5000) {
        if (telemetry.packetCount > 0 && millis() - telemetry.lastUpdate < 2000) {
            printTelemetry(&telemetry);
        }
        lastTelemetryPrint = millis();
    }
}