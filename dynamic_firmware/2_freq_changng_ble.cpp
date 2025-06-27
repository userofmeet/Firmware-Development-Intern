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

#define SAMPLE_WINDOW 20           // Increased for better smoothing
#define MONITOR_WINDOW_MS 300000   // 5 min
#define BLE_ON_MS 10000            // 10 sec

// Sampling intervals (ms) for each activity
#define RESTING_INTERVAL_MS 10000  // 0.1 Hz (10 s)
#define WALKING_INTERVAL_MS 1000   // 1 Hz (1 s)
#define RUNNING_INTERVAL_MS 500    // 2 Hz (0.5 s)

float acc_magnitude_buffer[SAMPLE_WINDOW];
int buffer_index = 0;
bool buffer_full = false;
SensorQMI8658 qmi;
IMUdata acc;

NimBLEServer *pServer = nullptr;
NimBLEAdvertising *adv = nullptr;
NimBLECharacteristic* activityChar = nullptr;
NimBLECharacteristic* echoChar = nullptr;

int totalSamples = 0;
int activeSamples = 0;
uint64_t windowStartTime = 0;
const char* current_activity = "Resting";
int activity_count = 0;
const char* pending_activity = nullptr;

// Calibration parameters
float scale_factor = 4.167f;
float offset_x = 0.0f, offset_y = 0.0f, offset_z = 0.0f;

float get_moving_average() {
    float sum = 0;
    for (int i = 0; i < SAMPLE_WINDOW; ++i) sum += acc_magnitude_buffer[i];
    return sum / SAMPLE_WINDOW;
}

const char* detect_activity(float net_mag) {
    if (net_mag < 0.1f) return "Resting";
    else if (net_mag < 0.5f) return "Walking";
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

void calibrate_sensor() {
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f, sum_mag = 0.0f;
    int samples = 100;
    int valid_samples = 0;

    ESP_LOGI(TAG, "Calibrating sensor... Keep device stationary for 1 second.");

    for (int i = 0; i < samples; ++i) {
        if (qmi.getDataReady()) {
            if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
                float raw_mag = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
                ESP_LOGI(TAG, "Pre-calibration Accel: %.2f, %.2f, %.2f, RawMag: %.2f",
                         acc.x, acc.y, acc.z, raw_mag);
                sum_x += acc.x;
                sum_y += acc.y;
                sum_z += acc.z;
                sum_mag += raw_mag;
                valid_samples++;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS); // 100 Hz
        }
    }

    if (valid_samples > 0) {
        offset_x = sum_x / valid_samples;
        offset_y = sum_y / valid_samples;
        offset_z = sum_z / valid_samples;
        float avg_mag = sum_mag / valid_samples;
        scale_factor = 1.0f / avg_mag;
        ESP_LOGI(TAG, "Calibration done: Offsets (x,y,z): %.3f, %.3f, %.3f, Scale: %.3f",
                 offset_x, offset_y, offset_z, scale_factor);
    } else {
        ESP_LOGE(TAG, "Calibration failed: No valid samples. Using fallback scale_factor: %.3f", scale_factor);
    }
}

void read_sensor_data(void* arg) {
    windowStartTime = esp_timer_get_time() / 1000ULL;
    int sample_interval_ms = RESTING_INTERVAL_MS;

    calibrate_sensor();

    float first_net_mag = 0.0f;
    bool first_reading_taken = false;

    while (1) {
        if (qmi.getDataReady()) {
            if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
                acc.x = (acc.x - offset_x) * scale_factor;
                acc.y = (acc.y - offset_y) * scale_factor;
                acc.z = (acc.z - offset_z) * scale_factor;

                float raw_mag = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
                float net_mag = fabs(raw_mag - 1.0f);

                ESP_LOGI(TAG, "Raw Accel: %.2f, %.2f, %.2f, RawMag: %.2f",
                         acc.x, acc.y, acc.z, raw_mag);

                if (!first_reading_taken) {
                    first_net_mag = net_mag;
                    for (int i = 0; i < SAMPLE_WINDOW; ++i) acc_magnitude_buffer[i] = first_net_mag;
                    first_reading_taken = true;
                    buffer_full = true;
                } else {
                    acc_magnitude_buffer[buffer_index++] = net_mag;
                    if (buffer_index >= SAMPLE_WINDOW) {
                        buffer_index = 0;
                        buffer_full = true;
                    }
                }

                if (buffer_full) {
                    float avg_mag = get_moving_average();
                    const char* activity = detect_activity(avg_mag);
                    ESP_LOGI(TAG, "Activity: %s, Accel: %.2f %.2f %.2f, RawMag: %.2f, AvgMag: %.2f",
                             activity, acc.x, acc.y, acc.z, raw_mag, avg_mag);

                    if (strcmp(activity, current_activity) != 0) {
                        int required_count = 3; // Default for other transitions
                        if (strcmp(current_activity, "Resting") == 0 && strcmp(activity, "Walking") == 0) {
                            required_count = 2; // Resting to Walking
                        } else if (strcmp(current_activity, "Walking") == 0 && strcmp(activity, "Running") == 0) {
                            required_count = 4; // Walking to Running
                        }

                        if (pending_activity == nullptr || strcmp(activity, pending_activity) != 0) {
                            pending_activity = activity;
                            activity_count = 1;
                        } else {
                            activity_count++;
                            if (activity_count >= required_count) {
                                current_activity = pending_activity;
                                activity_count = 0;
                                pending_activity = nullptr;

                                if (strcmp(current_activity, "Resting") == 0) {
                                    sample_interval_ms = RESTING_INTERVAL_MS;
                                } else if (strcmp(current_activity, "Walking") == 0) {
                                    sample_interval_ms = WALKING_INTERVAL_MS;
                                } else if (strcmp(current_activity, "Running") == 0) {
                                    sample_interval_ms = RUNNING_INTERVAL_MS;
                                }
                                ESP_LOGI(TAG, "Changed to %s, sampling interval: %d ms", current_activity, sample_interval_ms);
                            }
                        }
                    } else {
                        activity_count = 0;
                        pending_activity = nullptr;
                    }

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
                        float percentActive = (totalSamples > 0) ? (100.0f * activeSamples) / totalSamples : 0.0f;
                        ESP_LOGI(TAG, "5 min done: Active %.2f%%", percentActive);

                        if (percentActive >= 70.0f) {
                            start_ble();
                            vTaskDelay(BLE_ON_MS / portTICK_PERIOD_MS);
                            stop_ble();
                        }

                        windowStartTime = now;
                        totalSamples = 0;
                        activeSamples = 0;
                    }
                }
            }
        }
        vTaskDelay(sample_interval_ms / portTICK_PERIOD_MS);
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

    setup_sensor();
    xTaskCreate(read_sensor_data, "sensor_read_task", 4096, NULL, 10, NULL);
}
