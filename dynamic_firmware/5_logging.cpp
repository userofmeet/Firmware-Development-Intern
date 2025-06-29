#include <cmath>
#include <cstring>
#include <string>
#include <algorithm>
#include <fstream>
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_spiffs.h"
#include "esp_http_client.h"
#include "NimBLEDevice.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "SensorLib.h"
#include "SensorQMI8658.hpp"

#define WIFI_SSID "YourSSID"
#define WIFI_PASS "YourPassword"
#define LOG_FILE_PATH "/spiffs/log.txt"
#define SERVER_URL "http://192.168.1.100/upload" // enter the server URL 
#define UPLOAD_INTERVAL_MS (12ULL * 60 * 60 * 1000) // 12 hours so 12 * 60 * 60 * 1000 ms

#define COORD_SERVICE_UUID "180A"
#define RX_CHAR_UUID       "2A58"
#define ECHO_CHAR_UUID     "2A59"
#define ACTIVITY_CHAR_UUID "2A5E"

#define I2C_MASTER_SCL     14
#define I2C_MASTER_SDA     15
#define I2C_MASTER_NUM     I2C_NUM_0
#define QMI8658_ADDRESS    0x6B
#define AXP2101_ADDRESS    0x34
#define QMI8658_INT_PIN    13

#define SAMPLE_WINDOW 20
#define MONITOR_WINDOW_MS 300000    // 5 * 60 * 1000 ms
#define BLE_ON_MS 10000         // BLE on for 10 seconds
#define RESTING_10_MIN_MS 300000    // 10 * 60 * 1000 ms for deep sleep

#define RESTING_INTERVAL_MS 10000
#define WALKING_INTERVAL_MS 1000
#define RUNNING_INTERVAL_MS 500

static const char *TAG = "DOG_TRACKER_BLE";

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
uint64_t resting_start_time = 0;

float scale_factor = 4.167f;
float offset_x = 0.0f, offset_y = 0.0f, offset_z = 0.0f;

// SPIFFS Logging 
void log_event(const std::string& line) {
    FILE* f = fopen(LOG_FILE_PATH, "a");
    if (f) {
        fprintf(f, "%llu: %s\n", esp_timer_get_time() / 1000ULL, line.c_str());
        fclose(f);
    } else {
        ESP_LOGE(TAG, "Failed to open log file");
    }
}

// BLE logic 
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
        log_event("BLE started");
    }
}

void stop_ble() {
    if (adv->isAdvertising()) {
        adv->stop();
        ESP_LOGI(TAG, "BLE stopped");
        log_event("BLE stopped");
    }
}

// wifi and upload logic
void init_wifi() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

bool connect_wifi() {
    esp_wifi_connect();
    int retries = 10;
    wifi_ap_record_t ap_info;
    while (retries-- > 0) {
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            ESP_LOGI(TAG, "Connected to Wi-Fi");
            return true;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGE(TAG, "Wi-Fi connection failed");
    return false;
}

void upload_logs() {
    FILE* f = fopen(LOG_FILE_PATH, "r");
    if (!f) {
        ESP_LOGW(TAG, "No log file to upload");
        return;
    }

    fseek(f, 0, SEEK_END);
    size_t size = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::string body(size, '\0');
    fread(&body[0], 1, size, f);
    fclose(f);

    esp_http_client_config_t config = {};
    config.url = SERVER_URL;
    config.method = HTTP_METHOD_POST;
    config.timeout_ms = 5000;


    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "text/plain");
    esp_http_client_set_post_field(client, body.c_str(), body.length());

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK && esp_http_client_get_status_code(client) == 200) {
        ESP_LOGI(TAG, "Log upload successful");
        log_event("Log uploaded to server");
        remove(LOG_FILE_PATH);
    } else {
        ESP_LOGE(TAG, "Upload failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

void upload_log_task(void* arg) {
    while (1) {
        ESP_LOGI(TAG, "Starting 12-hr upload cycle");
        init_wifi();
        if (connect_wifi()) {
            upload_logs();
            esp_wifi_disconnect();
        }
        esp_wifi_stop();
        vTaskDelay(UPLOAD_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

// i2c and sensor initialization
void i2c_master_init() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

void axp2101_init() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP2101_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x10, true);
    i2c_master_write_byte(cmd, 0xFF, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AXP2101 init failed: %s", esp_err_to_name(ret));
    }
}

void setup_sensor() {
    i2c_master_init();
    axp2101_init();
    if (!qmi.begin(I2C_MASTER_NUM, QMI8658_ADDRESS, I2C_MASTER_SDA, I2C_MASTER_SCL)) {
        ESP_LOGE(TAG, "Sensor init failed.");
        vTaskDelete(NULL);
    }
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz,
                            SensorQMI8658::LPF_MODE_0, true);
    qmi.enableAccelerometer();
}

void setup_motion_wakeup() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << QMI8658_INT_PIN;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)QMI8658_INT_PIN, 1);
}

void calibrate_sensor() {
    float sum_x = 0, sum_y = 0, sum_z = 0, sum_mag = 0;
    int samples = 100, valid = 0;
    for (int i = 0; i < samples; ++i) {
        if (qmi.getDataReady() && qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
            float mag = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
            sum_x += acc.x; sum_y += acc.y; sum_z += acc.z; sum_mag += mag;
            valid++;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    if (valid > 0) {
        offset_x = sum_x / valid;
        offset_y = sum_y / valid;
        offset_z = sum_z / valid;
        scale_factor = 1.0f / (sum_mag / valid);
    }
}

// sensor reading task
void read_sensor_data(void* arg) {
    windowStartTime = esp_timer_get_time() / 1000ULL;
    calibrate_sensor();
    int sample_interval_ms = RESTING_INTERVAL_MS;
    float first_net_mag = 0.0f;
    bool first_reading = false;

    while (1) {
        if (qmi.getDataReady() && qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
            acc.x = (acc.x - offset_x) * scale_factor;
            acc.y = (acc.y - offset_y) * scale_factor;
            acc.z = (acc.z - offset_z) * scale_factor;

            float raw_mag = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
            float net_mag = fabs(raw_mag - 1.0f);

            if (!first_reading) {
                first_net_mag = net_mag;
                std::fill_n(acc_magnitude_buffer, SAMPLE_WINDOW, first_net_mag);
                first_reading = true;
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

                char log_line[128];
                snprintf(log_line, sizeof(log_line), "Activity=%s, Acc=%.2f,%.2f,%.2f",
                         activity, acc.x, acc.y, acc.z);
                log_event(log_line);

                if (strcmp(activity, current_activity) != 0) {
                    int required_count = 3;
                    if (strcmp(current_activity, "Resting") == 0 && strcmp(activity, "Walking") == 0)
                        required_count = 2;
                    else if (strcmp(current_activity, "Walking") == 0 && strcmp(activity, "Running") == 0)
                        required_count = 5;

                    if (pending_activity == nullptr || strcmp(activity, pending_activity) != 0) {
                        pending_activity = activity;
                        activity_count = 1;
                    } else {
                        activity_count++;
                        if (activity_count >= required_count) {
                            current_activity = pending_activity;
                            pending_activity = nullptr;
                            activity_count = 0;

                            if (strcmp(current_activity, "Resting") == 0) {
                                sample_interval_ms = RESTING_INTERVAL_MS;
                                resting_start_time = esp_timer_get_time() / 1000ULL;
                            } else {
                                sample_interval_ms = (strcmp(current_activity, "Walking") == 0) ? WALKING_INTERVAL_MS : RUNNING_INTERVAL_MS;
                                resting_start_time = 0;
                            }
                        }
                    }
                }

                totalSamples++;
                if (strcmp(activity, "Walking") == 0 || strcmp(activity, "Running") == 0)
                    activeSamples++;

                if (activityChar && adv->isAdvertising()) {
                    activityChar->setValue(activity);
                    activityChar->notify();
                }

                uint64_t now = esp_timer_get_time() / 1000ULL;
                if (now - windowStartTime >= MONITOR_WINDOW_MS) {
                    float percent = (totalSamples > 0) ? (100.0f * activeSamples / totalSamples) : 0.0f;
                    if (percent >= 70.0f) {
                        start_ble();
                        vTaskDelay(BLE_ON_MS / portTICK_PERIOD_MS);
                        stop_ble();
                        resting_start_time = 0;
                    }
                    windowStartTime = now;
                    totalSamples = 0;
                    activeSamples = 0;
                }

                if (strcmp(current_activity, "Resting") == 0 && resting_start_time &&
                    (now - resting_start_time) > RESTING_10_MIN_MS) {
                    log_event("Deep Sleep triggered");
                    setup_motion_wakeup();
                    esp_deep_sleep_start();
                }
            }
        }
        vTaskDelay(sample_interval_ms / portTICK_PERIOD_MS);
    }
}

class RxCallbacks : public NimBLECharacteristicCallbacks {
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
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

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
    xTaskCreate(read_sensor_data, "sensor_read_task", 8192, NULL, 10, NULL);
    xTaskCreate(upload_log_task, "upload_log_task", 8192, NULL, 5, NULL);
}
