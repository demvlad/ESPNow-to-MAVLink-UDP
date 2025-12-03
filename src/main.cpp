#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2  // GPIO2 для большинства ESP32
#endif

// Конфигурация
#define QUEUE_SIZE 20
#define PACKET_TIMEOUT_MS 200
#define TELEMETRY_TIMEOUT_MS 3000

// MAC адрес как в рабочем коде
uint8_t UID[6] = {78, 82, 166, 251, 35, 234}; // {0x4E, 0x52, 0xA6, 0xFB, 0x23, 0xEA}

// Структура ELRSHeader - ИСПРАВЛЕНА согласно логу
#pragma pack(push, 1)
struct ELRSHeader {
    uint8_t sync[3];     // 0x24 0x58 0x3C
    uint8_t flags;       // 0x00
    uint16_t packetId;   // 0x00 0x11 (little endian)
    uint8_t sequence;    // 0x00
    uint8_t reserved;    // 0x09
    uint8_t crsfAddr;    // 0xEA - ВАЖНО!
    // ДАЛЬШЕ ИДУТ CRSF ДАННЫЕ, НЕ ЗАГОЛОВОК ELRS!
};
#pragma pack(pop)

// Структуры CRSF
typedef struct {
    uint8_t deviceAddress;  // Обычно 0xEA
    uint8_t frameLength;    // Длина фрейма (включая type, payload и crc)
    uint8_t type;           // Тип пакета CRSF
    uint8_t payload[0];     // Данные пакета
} crsfFrameDef_t;

typedef crsfFrameDef_t* crsfFrame_t;

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

// Структуры данных CRSF
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

// Структура для очереди
typedef struct {
    uint8_t data[300];
    uint8_t len;
    int8_t rssi;
    uint32_t timestamp;
} ESPNowPacket;

// Телеметрия
struct TelemetryData {
    uint16_t channels[16];
    uint16_t voltage_raw;      // Сырое значение напряжения
    uint16_t current_raw;      // Сырое значение тока  
    uint32_t capacity_raw;     // Сырое значение емкости
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
    int8_t uplinkRSSI1;
    int8_t uplinkRSSI2;
    uint8_t uplinkLinkQuality;
    int8_t uplinkSNR;
    uint8_t activeAntenna;
    uint8_t rfMode;
    uint8_t uplinkTXPower;
    int8_t downlinkRSSI;
    uint8_t downlinkLinkQuality;
    int8_t downlinkSNR;
    uint32_t packetCount;
    uint32_t crsfPackets[256];
    unsigned long lastUpdate;
    uint32_t lastChannelUpdate;
    int8_t currentRSSI;
    uint8_t linkQuality;
};

// Глобальные объекты
QueueHandle_t packetQueue = NULL;
TelemetryData telemetry = {0};

// Прототипы
void IRAM_ATTR OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
void processingTask(void* parameter);
bool parseELRSPacket(const uint8_t *data, int len, TelemetryData* telemetry, int8_t rssi);
uint16_t unpack11bit(const uint8_t* data, int channel);
uint8_t crsfCRC(const uint8_t* data, uint8_t len);

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
    static uint32_t allPackets = 0;
    allPackets++;
    
    // Выводим структуру первых 5 пакетов
    if (allPackets <= 5) {
        Serial.printf("\n[RAW PACKET #%lu] Len: %d\n", allPackets, data_len);
        Serial.print("  All bytes: ");
        for(int i = 0; i < min(data_len, 24); i++) {
            Serial.printf("%02X ", data[i]);
            if (i == 9 || i == 19) Serial.print("| "); // Разделители
        }
        if (data_len > 24) Serial.print("...");
        Serial.println();
        
        // Проверяем ELRS sync
        if (data_len >= 3) {
            Serial.printf("  ELRS sync: %02X %02X %02X -> %s\n",
                         data[0], data[1], data[2],
                         (data[0]==0x24&&data[1]==0x58&&data[2]==0x3C)?"OK":"NOT ELRS");
        }
        
        // Показываем байт 8 (crsfAddr)
        if (data_len >= 9) {
            Serial.printf("  Byte 8 (crsfAddr): 0x%02X\n", data[8]);
        }
        
        // Показываем байт 10 (начало CRSF данных)
        if (data_len >= 11) {
            Serial.printf("  Byte 10 (CRSF type): 0x%02X\n", data[10]);
        }
    }
    
    // Быстрая проверка
    if (data_len < 11 || data_len > 300 || packetQueue == NULL) return;
    
    // Проверка ELRS синхробайтов
    if (data[0] != 0x24 || data[1] != 0x58 || data[2] != 0x3C) return;
    
    ESPNowPacket packet;
    packet.len = data_len;
    packet.timestamp = micros();
    
    // RSSI - заглушка
    packet.rssi = -70;
    
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
    
    // Отладочный вывод первых пакетов
    static uint32_t packetCount = 0;
    packetCount++;
    if (packetCount <= 3) {
        Serial.printf("[ESPNOW] Packet #%lu received, len: %d\n", packetCount, data_len);
    }
}

// Функция проверки CRC (из ExpressLRS)
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

// Функция для распаковки 11-битных каналов
uint16_t unpack11bit(const uint8_t* data, int channel) {
    uint32_t bitIndex = channel * 11;
    uint32_t byteIndex = bitIndex / 8;
    uint32_t bitOffset = bitIndex % 8;
    
    uint32_t value = ((uint32_t)data[byteIndex] << 16) |
                     ((uint32_t)data[byteIndex + 1] << 8) |
                     ((uint32_t)data[byteIndex + 2]);
    
    value >>= bitOffset;
    value &= 0x7FF;
    
    return (uint16_t)value;
}

// Парсинг пакета - ОСНОВНОЕ ИСПРАВЛЕНИЕ
bool parseELRSPacket(const uint8_t *data, int len, TelemetryData* telemetry, int8_t rssi) {
    // Быстрые проверки
    if (len < 11) return false;
    if (data[0] != 0x24 || data[1] != 0x58 || data[2] != 0x3C) return false;
    
    // CRSF данные начинаются с байта 8
    const uint8_t *crsfData = data + 8;
    int crsfLen = len - 8;
    if (crsfLen < 4) return false;
    
    // Чтение заголовка
    uint8_t frame_len = crsfData[1];
    uint8_t frame_type = crsfData[2];
    
    if (frame_len < 3 || frame_len > crsfLen) return false;
    
    uint8_t payload_len = frame_len - 2;
    const uint8_t* payload = crsfData + 3;
    
    // Обновление статистики
    telemetry->packetCount++;
    if (frame_type < 256) telemetry->crsfPackets[frame_type]++;
    telemetry->lastUpdate = millis();
    telemetry->currentRSSI = rssi;
    
    // Обработка пакетов
    switch (frame_type) {
        case 0x08: // Battery
            if (payload_len >= 8) {
                uint16_t v = (payload[0] << 8) | payload[1];
                uint16_t c = (payload[2] << 8) | payload[3];
                telemetry->voltage = v * 0.1f;
                telemetry->current = c * 0.1f;
                telemetry->capacity = (payload[4] << 16) | (payload[5] << 8) | payload[6];
                telemetry->batteryRemaining = payload[7];
            }
            break;
            
        case 0x1E: // Attitude
            if (payload_len >= 6) {
                int16_t p = (int16_t)((payload[0] << 8) | payload[1]);
                int16_t r = (int16_t)((payload[2] << 8) | payload[3]);
                int16_t y = (int16_t)((payload[4] << 8) | payload[5]);
                telemetry->pitch = p * 0.00572957795f; // rad/10000 → градусы
                telemetry->roll = r * 0.00572957795f;
                telemetry->yaw = y * 0.00572957795f;
            }
            break;
            
        case 0x14: // Link Statistics
            if (payload_len >= 10) {
                telemetry->uplinkRSSI1 = (payload[0] / 2) - 120;
                telemetry->uplinkRSSI2 = (payload[1] / 2) - 120;
                telemetry->uplinkLinkQuality = payload[2];
                telemetry->uplinkSNR = (int8_t)payload[3];
                telemetry->downlinkRSSI = (payload[7] / 2) - 120;
                telemetry->downlinkLinkQuality = payload[8];
                telemetry->downlinkSNR = (int8_t)payload[9];
            }
            break;
            
        case 0x21: // Flight Mode
            if (payload_len >= 1) {
                int l = payload_len < 16 ? payload_len : 16;
                memcpy(telemetry->flightMode, payload, l);
                telemetry->flightMode[l] = '\0';
            }
            break;
    }
    
    return true;
}

// Задача обработки
void processingTask(void* parameter) {
    ESPNowPacket packet;
    uint32_t packetsProcessed = 0;
    uint32_t parseErrors = 0;
    uint32_t lastStatsTime = 0;
    
    Serial.println("[TASK] Processing task started on Core 1");
    
    while (1) {
        if (xQueueReceive(packetQueue, &packet, portMAX_DELAY) == pdTRUE) {
            packetsProcessed++;
            
            digitalWrite(LED_BUILTIN, HIGH);
            
            // Проверка таймаута
            if ((micros() - packet.timestamp) / 1000 > PACKET_TIMEOUT_MS) {
                digitalWrite(LED_BUILTIN, LOW);
                continue;
            }
            
            // Отладка первых пакетов
            if (packetsProcessed <= 5) {
                Serial.printf("\n[PROCESSING #%lu] Len:%d\n", packetsProcessed, packet.len);
            }
            
            // Парсинг
            if (!parseELRSPacket(packet.data, packet.len, &telemetry, packet.rssi)) {
                parseErrors++;
                if (packetsProcessed <= 5) {
                    Serial.println("  PARSE FAILED");
                }
            } else {
                if (packetsProcessed <= 5) {
                    Serial.println("  PARSE OK");
                }
            }
            
            // Вывод статистики
            if (millis() - lastStatsTime >= 5000) {
                Serial.printf("[STATS] Total:%lu, Errors:%lu, Queue:%d/%d\n",
                            packetsProcessed, parseErrors,
                            uxQueueMessagesWaiting(packetQueue),
                            QUEUE_SIZE);
                
                // Вывод типов пакетов
                Serial.print("  Packet types: ");
                bool first = true;
                for (int i = 0; i < 256; i++) {
                    if (telemetry.crsfPackets[i] > 0) {
                        if (!first) Serial.print(", ");
                        Serial.printf("0x%02X:%lu", i, telemetry.crsfPackets[i]);
                        first = false;
                    }
                }
                if (first) Serial.print("None");
                Serial.println();
                
                lastStatsTime = millis();
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
    
    delay(100);
}