#include <cmath>
#include <cstring>
#include <string>
#include <algorithm>
#include <chrono>

#include "esp_log.h"
#include "nvs_flash.h"
#include "NimBLEDevice.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "SensorLib.h"
#include "SensorQMI8658.hpp"

static const char *TAG = "DOG_TRACKER_BLE";

// UUIDs
#define COORD_SERVICE_UUID "180A"
#define RX_CHAR_UUID       "2A58"
#define ECHO_CHAR_UUID     "2A59"
#define DIST_CHAR_UUID     "2A5A"
#define INFO_CHAR_UUID     "2A5B"
#define SPEED_CHAR_UUID    "2A5C"
#define STEPS_CHAR_UUID    "2A5D"

static constexpr int kAdvIntervalMs = 100;

// I2C + Sensor
#define I2C_MASTER_SCL     14
#define I2C_MASTER_SDA     15
#define I2C_MASTER_NUM     I2C_NUM_0
#define QMI8658_ADDRESS    0x6B

#define SAMPLE_WINDOW 10
float acc_magnitude_buffer[SAMPLE_WINDOW];
int buffer_index = 0;
SensorQMI8658 qmi;
IMUdata acc, gyr;

// BLE state
static float prevLat = NAN, prevLon = NAN;
static std::chrono::steady_clock::time_point prevTime;
static std::string dogBreed = "";
static int dogAge = 0;
static float dogSize = 0.0f;
static float stepSize = 0.0f;

// ------------------ Utility -------------------
static float deg2rad(float deg) {
    return deg * M_PI / 180.0f;
}

static float haversine_m(float lat1, float lon1, float lat2, float lon2) {
    const float R = 6371000.0f;
    float dLat = deg2rad(lat2 - lat1);
    float dLon = deg2rad(lon2 - lon1);
    float a = pow(sin(dLat / 2), 2) +
              cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
              pow(sin(dLon / 2), 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

float estimateStepSize(const std::string &breed, int age, float size) {
    float baseStep = 0.60f;
    if (breed == "Labrador") baseStep = 0.75f;
    else if (breed == "Beagle") baseStep = 0.50f;
    else if (breed == "GermanShepherd") baseStep = 0.80f;
    else if (breed == "GoldenRetriever") baseStep = 0.78f;
    else if (breed == "Bulldog") baseStep = 0.45f;
    else if (breed == "Poodle") baseStep = 0.60f;
    else if (breed == "Boxer") baseStep = 0.35f;
    else if (breed == "Husky") baseStep = 0.70f;
    else if (breed == "Cockerspaniel") baseStep = 0.48f;
    else if (breed == "Dachshund") baseStep = 0.35f;
    else if (breed == "Chihuahua") baseStep = 0.30f;
    else if (breed == "Greatdane") baseStep = 1.00f;
    else if (breed == "Doberman") baseStep = 0.78f;
    else if (breed == "Rottweiler") baseStep = 0.80f;
    else if (breed == "Bordercollie") baseStep = 0.60f;
    else if (breed == "Vizsla") baseStep = 0.70f;
    else if (breed == "Greyhound") baseStep = 0.88f;

    if (size > 0.10f) baseStep *= size / 0.5f;
    if (age < 1) baseStep *= 0.7f;
    else if (age > 8) baseStep *= 0.85f;

    return baseStep;
}

// ------------------ BLE Classes -------------------
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
        ESP_LOGI(TAG, "Client connected");
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
        ESP_LOGI(TAG, "Client disconnected, restarting advertising");
        NimBLEDevice::getAdvertising()->start();
    }
};

class InfoCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChr, NimBLEConnInfo&) override {
        std::string raw = pChr->getValue();
        if (raw.empty()) return;

        raw.erase(std::remove_if(raw.begin(), raw.end(), ::isspace), raw.end());
        char breedBuf[32] = {0}; int age = 0; float size = 0.0f;
        int ret = std::sscanf(raw.c_str(), "%31[^,],%d,%f", breedBuf, &age, &size);
        if (ret != 3) return;

        dogBreed = std::string(breedBuf);
        dogAge = age;
        dogSize = size;
        stepSize = estimateStepSize(dogBreed, dogAge, dogSize);

        ESP_LOGI(TAG, "Dog info: %s, %d yrs, %.2f m, StepSize: %.2f m",
                 dogBreed.c_str(), dogAge, dogSize, stepSize);
    }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
    NimBLECharacteristic *echoChar_, *distChar_, *speedChar_, *stepsChar_;
public:
    RxCallbacks(NimBLECharacteristic *echoC, NimBLECharacteristic *distC,
                NimBLECharacteristic *speedC, NimBLECharacteristic *stepsC)
        : echoChar_(echoC), distChar_(distC), speedChar_(speedC), stepsChar_(stepsC) {}

    void onWrite(NimBLECharacteristic *pChr, NimBLEConnInfo&) override {
        std::string raw = pChr->getValue();
        raw.erase(std::remove_if(raw.begin(), raw.end(), ::isspace), raw.end());
        float lat, lon;
        if (std::sscanf(raw.c_str(), "%f,%f", &lat, &lon) != 2) return;

        echoChar_->setValue(raw); echoChar_->notify();
        ESP_LOGI(TAG, "Coordinates: %f, %f", lat, lon);

        float dist = 0.0f, speed = 0.0f; int steps = 0;
        auto now = std::chrono::steady_clock::now();

        if (!std::isnan(prevLat) && !std::isnan(prevLon)) {
            dist = haversine_m(prevLat, prevLon, lat, lon);
            float timeSec = std::chrono::duration<float>(now - prevTime).count();
            if (timeSec > 0.001f) speed = dist / timeSec;
            if (stepSize > 0) steps = static_cast<int>(dist / stepSize);
        }

        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.2f", dist); distChar_->setValue(buf); distChar_->notify();
        std::snprintf(buf, sizeof(buf), "%.2f", speed); speedChar_->setValue(buf); speedChar_->notify();
        std::snprintf(buf, sizeof(buf), "%d", steps); stepsChar_->setValue(buf); stepsChar_->notify();

        ESP_LOGI(TAG, "Dist: %.2f m, Speed: %.2f m/s, Steps: %d", dist, speed, steps);

        prevLat = lat; prevLon = lon; prevTime = now;
    }
};

// ------------------ Sensor Code -------------------
float get_moving_average() {
    float sum = 0;
    for (int i = 0; i < SAMPLE_WINDOW; ++i) sum += acc_magnitude_buffer[i];
    return sum / SAMPLE_WINDOW;
}

const char* detect_activity(float net_magnitude) {
    if (net_magnitude < 0.1f) return "Resting";
    else if (net_magnitude < 0.5f) return "Walking";
    else return "Running";
}

void i2c_master_init() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void setup_sensor() {
    i2c_master_init();
    if (!qmi.begin(I2C_MASTER_NUM, QMI8658_ADDRESS, I2C_MASTER_SDA, I2C_MASTER_SCL)) {
        ESP_LOGE(TAG, "Sensor init failed.");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "Sensor ID: %x", qmi.getChipID());

    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz,
                            SensorQMI8658::LPF_MODE_0, true);
    qmi.configGyroscope(SensorQMI8658::GYR_RANGE_64DPS, SensorQMI8658::GYR_ODR_896_8Hz,
                        SensorQMI8658::LPF_MODE_3, true);
    qmi.enableAccelerometer();
    qmi.enableGyroscope();
}

void read_sensor_data(void* arg) {
    while (1) {
        if (qmi.getDataReady()) {
            if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
                float raw_mag = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
                float net_mag = fabs(raw_mag - 1.0f);
                acc_magnitude_buffer[buffer_index++] = net_mag;
                if (buffer_index >= SAMPLE_WINDOW) buffer_index = 0;

                float avg_mag = get_moving_average();
                const char* activity = detect_activity(avg_mag);
                ESP_LOGI(TAG, "Activity: %s, Accel: %.2f %.2f %.2f", activity, acc.x, acc.y, acc.z);
            }
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

// ------------------ Main Entry -------------------
extern "C" void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    NimBLEDevice::init("MJ-DogTracker");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    NimBLEDevice::setSecurityAuth(false, false, true);  // Ensure advertising works

    NimBLEServer *pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService *pSvc = pServer->createService(COORD_SERVICE_UUID);

    auto *rxChar = pSvc->createCharacteristic(RX_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
    auto *echoChar = pSvc->createCharacteristic(ECHO_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    auto *distChar = pSvc->createCharacteristic(DIST_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    auto *speedChar = pSvc->createCharacteristic(SPEED_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    auto *stepsChar = pSvc->createCharacteristic(STEPS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    auto *infoChar = pSvc->createCharacteristic(INFO_CHAR_UUID, NIMBLE_PROPERTY::WRITE);

    infoChar->setCallbacks(new InfoCallbacks());
    rxChar->setCallbacks(new RxCallbacks(echoChar, distChar, speedChar, stepsChar));

    pSvc->start();

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->setAppearance(0x03C0);
    adv->setMinInterval(kAdvIntervalMs / 0.625);
    adv->setMaxInterval(kAdvIntervalMs / 0.625);
    adv->addServiceUUID(COORD_SERVICE_UUID);

    // 🔧 Correct way for NimBLE
    NimBLEDevice::init("MJ-DogTracker"); // Set internal device name
    //NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();

    // Advertisement data
    NimBLEAdvertisementData advData;
    advData.setCompleteServices(NimBLEUUID(COORD_SERVICE_UUID));  // your custom service
    advData.setAppearance(0x03C0);
    adv->setAdvertisementData(advData);

    // Scan response data with device name
    NimBLEAdvertisementData scanRespData;
    scanRespData.setName("MJ-DogTracker");
    adv->setScanResponseData(scanRespData);

    adv->setMinInterval(kAdvIntervalMs / 0.625);  // convert ms to BLE units
    adv->setMaxInterval(kAdvIntervalMs / 0.625);

    adv->start();
    ESP_LOGI(TAG, "BLE started with name MJ-DogTracker");

    setup_sensor();
    xTaskCreate(read_sensor_data, "sensor_read_task", 4096, NULL, 10, NULL);
}
