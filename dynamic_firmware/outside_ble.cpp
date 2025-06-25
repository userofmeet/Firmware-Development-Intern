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

#define COORD_SERVICE_UUID "180A"
#define RX_CHAR_UUID       "2A58"
#define ECHO_CHAR_UUID     "2A59"
#define ACTIVITY_CHAR_UUID "2A5E"

#define I2C_MASTER_SCL     14
#define I2C_MASTER_SDA     15
#define I2C_MASTER_NUM     I2C_NUM_0
#define QMI8658_ADDRESS    0x6B

#define SAMPLE_WINDOW 10
#define SAMPLE_INTERVAL_MS 200
#define MONITOR_WINDOW_MS 300000  // 5 min
#define BLE_ON_MS 10000           // 10 sec

float acc_magnitude_buffer[SAMPLE_WINDOW];
int buffer_index = 0;
SensorQMI8658 qmi;
IMUdata acc;

NimBLEServer *pServer = nullptr;
NimBLEAdvertising *adv = nullptr;
NimBLECharacteristic* activityChar = nullptr;
NimBLECharacteristic* echoChar = nullptr;

int totalSamples = 0;
int activeSamples = 0;
uint64_t windowStartTime = 0;

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

void start_ble() {
    if (!adv->isAdvertising()) {
        adv->start();
        ESP_LOGI(TAG, "BLE started");
    }
}

void stop_ble() {
    if (adv->isAdvertising()) {
        adv->stop();
        ESP_LOGI(TAG, "BLE stopped");
    }
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
    qmi.enableAccelerometer();
}

void read_sensor_data(void* arg) {
    windowStartTime = esp_timer_get_time() / 1000ULL;  // ms
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

                totalSamples++;
                if (strcmp(activity, "Walking") == 0 || strcmp(activity, "Running") == 0) {
                    activeSamples++;
                }

                if (activityChar && adv->isAdvertising()) {
                    activityChar->setValue(activity);
                    activityChar->notify();
                }

                uint64_t now = esp_timer_get_time() / 1000ULL;
                if (now - windowStartTime >= MONITOR_WINDOW_MS) {
                    float percentActive = (100.0f * activeSamples) / totalSamples;
                    ESP_LOGI(TAG, "5 min done: Active %.2f%%", percentActive);

                    if (percentActive >= 70.0f) {
                        start_ble();
                        vTaskDelay(BLE_ON_MS / portTICK_PERIOD_MS);
                        stop_ble();
                    }

                    // Reset window
                    windowStartTime = now;
                    totalSamples = 0;
                    activeSamples = 0;
                }
            }
        }
        vTaskDelay(SAMPLE_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

class RxCallbacks : public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic *pChr, NimBLEConnInfo&) override {
        std::string raw = pChr->getValue();
        raw.erase(std::remove_if(raw.begin(), raw.end(), ::isspace), raw.end());
        echoChar->setValue(raw);
        echoChar->notify();
        ESP_LOGI(TAG, "Echoed coordinates: %s", raw.c_str());
    }
};

extern "C" void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    NimBLEDevice::init("MJ");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    NimBLEDevice::setSecurityAuth(false, false, true);

    pServer = NimBLEDevice::createServer();

    NimBLEService *pSvc = pServer->createService(COORD_SERVICE_UUID);

    auto *rxChar = pSvc->createCharacteristic(RX_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
    echoChar = pSvc->createCharacteristic(ECHO_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    activityChar = pSvc->createCharacteristic(ACTIVITY_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    rxChar->setCallbacks(new RxCallbacks());

    pSvc->start();

    adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(COORD_SERVICE_UUID);

    // BLE will be started/stopped dynamically

    setup_sensor();
    xTaskCreate(read_sensor_data, "sensor_read_task", 4096, NULL, 10, NULL);
}
  
