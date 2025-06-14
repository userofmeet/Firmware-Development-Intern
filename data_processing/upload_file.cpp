// dog_tracker.cpp

#include <cmath>
#include <cstring>
#include <string>
#include <algorithm>
#include <chrono>
#include <fstream>

#include "esp_log.h"
#include "nvs_flash.h"
#include "NimBLEDevice.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_spiffs.h"

#include "SensorLib.h"
#include "SensorQMI8658.hpp"

static const char *TAG = "DOG_TRACKER_BLE";

// BLE UUIDs// BLE UUIDs (Custom 128-bit UUIDs to avoid SIG conflicts)
#define COORD_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RX_CHAR_UUID       "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define ECHO_CHAR_UUID     "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define DIST_CHAR_UUID     "6e400004-b5a3-f393-e0a9-e50e24dcca9e"
#define SPEED_CHAR_UUID    "6e400005-b5a3-f393-e0a9-e50e24dcca9e"
#define STEPS_CHAR_UUID    "6e4000006-b5a3-f393-e0a9-e50e24dcca9e"
#define ACTIVITY_CHAR_UUID "6e400007-b5a3-f393-e0a9-e50e24dcca9e"
#define LOG_TRIGGER_UUID   "6e400008-b5a3-f393-e0a9-e50e24dcca9e"
#define LOG_CHAR_UUID      "6e400009-b5a3-f393-e0a9-e50e24dcca9e"
#define INFO_CHAR_UUID     "6e40000A-b5a3-f393-e0a9-e50e24dcca9e"
#define LOG_CHUNK_SIZE     512

static constexpr const char* LOG_FILE = "/spiffs/dog_activity_log.txt";

// I2C Config
#define I2C_MASTER_SCL 14
#define I2C_MASTER_SDA 15
#define I2C_MASTER_NUM I2C_NUM_0
#define QMI8658_ADDRESS 0x6B

#define SAMPLE_WINDOW 10
float acc_buffer[SAMPLE_WINDOW];
int acc_idx = 0;
SensorQMI8658 qmi;
IMUdata acc, gyr;

// State
static float prevLat = NAN, prevLon = NAN;
static std::chrono::steady_clock::time_point prevTime;
static std::string dogBreed = "";
static int dogAge = 0;
static float dogSize = 0.0f;
static float stepSize = 0.0f;

NimBLECharacteristic *activityChar = nullptr;
NimBLECharacteristic *logChar = nullptr;

float deg2rad(float deg) {
    return deg * M_PI / 180.0f;
}

float haversine_m(float lat1, float lon1, float lat2, float lon2) {
    const float R = 6371000.0f;
    float dLat = deg2rad(lat2 - lat1);
    float dLon = deg2rad(lon2 - lon1);
    float a = pow(sin(dLat / 2), 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * pow(sin(dLon / 2), 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

float estimateStepSize(const std::string &breed, int age, float size) {
    float base = 0.6f;
    if (breed == "Labrador") base = 0.75f;
    else if (breed == "Beagle") base = 0.50f;
    else if (breed == "GermanShepherd") base = 0.80f;
    else if (breed == "GoldenRetriever") base = 0.78f;
    else if (breed == "Bulldog") base = 0.45f;
    else if (breed == "Poodle") base = 0.60f;
    else if (breed == "Boxer") base = 0.35f;
    else if (breed == "Husky") base = 0.70f;
    else if (breed == "Dachshund") base = 0.35f;
    else if (breed == "Chihuahua") base = 0.30f;
    else if (breed == "Greatdane") base = 1.00f;
    else if (breed == "Doberman") base = 0.78f;
    else if (breed == "Rottweiler") base = 0.80f;
    else if (breed == "Bordercollie") base = 0.60f;
    else if (breed == "Vizsla") base = 0.70f;
    else if (breed == "Greyhound") base = 0.88f;

    if (size > 0.10f) base *= size / 0.5f;
    if (age < 1) base *= 0.7f;
    else if (age > 8) base *= 0.85f;

    return base;
}

void i2c_master_init() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA;
    conf.scl_io_num = I2C_MASTER_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void setup_sensor() {
    i2c_master_init();
    if (!qmi.begin(I2C_MASTER_NUM, QMI8658_ADDRESS, I2C_MASTER_SDA, I2C_MASTER_SCL)) {
        ESP_LOGE(TAG, "Sensor init failed");
        vTaskDelete(NULL);
    }

    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_0, true);
    qmi.configGyroscope(SensorQMI8658::GYR_RANGE_64DPS, SensorQMI8658::GYR_ODR_896_8Hz, SensorQMI8658::LPF_MODE_3, true);
    qmi.enableAccelerometer();
    qmi.enableGyroscope();
}

float get_moving_average() {
    float sum = 0;
    for (int i = 0; i < SAMPLE_WINDOW; i++) sum += acc_buffer[i];
    return sum / SAMPLE_WINDOW;
}

const char* detect_activity(float net) {
    if (net < 0.1f) return "Resting";
    else if (net < 0.5f) return "Walking";
    return "Running";
}

void log_to_file(const char* activity, float x, float y, float z) {
    FILE* f = fopen(LOG_FILE, "a");
    if (!f) return;
    time_t now; time(&now);
    fprintf(f, "%lld,%s,%.2f,%.2f,%.2f\n", (long long)now, activity, x, y, z);
    fclose(f);
}

void read_sensor_data(void*) {
    while (1) {
        if (qmi.getDataReady() && qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
            float mag = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
            float net = fabs(mag - 1.0f);
            acc_buffer[acc_idx++] = net;
            if (acc_idx >= SAMPLE_WINDOW) acc_idx = 0;

            float avg = get_moving_average();
            const char* activity = detect_activity(avg);
            ESP_LOGI(TAG, "Activity: %s, Acc: %.2f %.2f %.2f", activity, acc.x, acc.y, acc.z);

            if (activityChar) {
                activityChar->setValue(activity);
                activityChar->notify();
            }

            log_to_file(activity, acc.x, acc.y, acc.z);
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void setup_spiffs() {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = nullptr,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
}

// BLE Callback Classes
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
        ESP_LOGI(TAG, "Client connected");
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
        ESP_LOGI(TAG, "Client disconnected");
        NimBLEDevice::getAdvertising()->start();
    }
};

class InfoCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChr, NimBLEConnInfo&) override {
        std::string raw = pChr->getValue();
        if (raw.empty()) return;

        raw.erase(std::remove_if(raw.begin(), raw.end(), ::isspace), raw.end());
        char breedBuf[32]; int age; float size;
        if (std::sscanf(raw.c_str(), "%31[^,],%d,%f", breedBuf, &age, &size) != 3) return;

        dogBreed = std::string(breedBuf); dogAge = age; dogSize = size;
        stepSize = estimateStepSize(dogBreed, dogAge, dogSize);

        ESP_LOGI(TAG, "Dog: %s, Age: %d, Size: %.2f, Step: %.2f",
                 dogBreed.c_str(), dogAge, dogSize, stepSize);
    }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
    NimBLECharacteristic *echo_, *dist_, *speed_, *steps_;
public:
    RxCallbacks(NimBLECharacteristic* e, NimBLECharacteristic* d, NimBLECharacteristic* s, NimBLECharacteristic* st)
        : echo_(e), dist_(d), speed_(s), steps_(st) {}

    void onWrite(NimBLECharacteristic *pChr, NimBLEConnInfo&) override {
        std::string raw = pChr->getValue();
        raw.erase(std::remove_if(raw.begin(), raw.end(), ::isspace), raw.end());
        float lat, lon;
        if (std::sscanf(raw.c_str(), "%f,%f", &lat, &lon) != 2) return;

        echo_->setValue(raw); echo_->notify();
        float dist = 0.0f, speed = 0.0f; int steps = 0;
        auto now = std::chrono::steady_clock::now();

        if (!std::isnan(prevLat) && !std::isnan(prevLon)) {
            dist = haversine_m(prevLat, prevLon, lat, lon);
            float secs = std::chrono::duration<float>(now - prevTime).count();
            if (secs > 0.001f) speed = dist / secs;
            if (stepSize > 0) steps = static_cast<int>(dist / stepSize);
        }

        char buf[32];
        snprintf(buf, sizeof(buf), "%.2f", dist); dist_->setValue(buf); dist_->notify();
        snprintf(buf, sizeof(buf), "%.2f", speed); speed_->setValue(buf); speed_->notify();
        snprintf(buf, sizeof(buf), "%d", steps); steps_->setValue(buf); steps_->notify();

        prevLat = lat; prevLon = lon; prevTime = now;

        ESP_LOGI(TAG, "Coords: %.5f, %.5f | Dist: %.2f m | Speed: %.2f m/s | Steps: %d", lat, lon, dist, speed, steps);

    }
};

class LogCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChr, NimBLEConnInfo&) override {
        ESP_LOGI(TAG, "Log transfer started");

        FILE* f = fopen(LOG_FILE, "r");
        if (!f) {
            ESP_LOGE(TAG, "Failed to open log file");
            return;
        }

        char line[256];
        int lineCount = 0;

        while (fgets(line, sizeof(line), f)) {
            logChar->setValue(line);  // safe because 1 line ~ 50-70 bytes
            logChar->notify();
            vTaskDelay(120 / portTICK_PERIOD_MS);  // Give time to BLE client
            lineCount++;
        }

        fclose(f);

        // Send END marker to confirm complete transfer
        logChar->setValue("[[END]]");
        logChar->notify();

        ESP_LOGI(TAG, "Sent %d lines from log", lineCount);
    }
};

// Main
extern "C" void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    setup_spiffs();

    NimBLEDevice::init("MJ");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    NimBLEDevice::setSecurityAuth(false, false, true);
    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    NimBLEService *svc = server->createService(COORD_SERVICE_UUID);
    auto *rx = svc->createCharacteristic(RX_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
    auto *echo = svc->createCharacteristic(ECHO_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    auto *dist = svc->createCharacteristic(DIST_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    auto *speed = svc->createCharacteristic(SPEED_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    auto *steps = svc->createCharacteristic(STEPS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    auto *info = svc->createCharacteristic(INFO_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
    auto *logTrigger = svc->createCharacteristic(LOG_TRIGGER_UUID, NIMBLE_PROPERTY::WRITE);
    logChar = svc->createCharacteristic(LOG_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    activityChar = svc->createCharacteristic(ACTIVITY_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    rx->setCallbacks(new RxCallbacks(echo, dist, speed, steps));
    info->setCallbacks(new InfoCallbacks());
    logTrigger->setCallbacks(new LogCallbacks());

    svc->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(COORD_SERVICE_UUID);
    adv->setAppearance(0x03C0);
    adv->setName("MJ");  // ← This is valid in NimBLE
    adv->start();


    ESP_LOGI(TAG, "BLE Advertising started");

    setup_sensor();
    xTaskCreate(read_sensor_data, "sensor_read", 8192, NULL, 10, NULL);
}
