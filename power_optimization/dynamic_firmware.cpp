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
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_http_client.h"

// Configuration
static const char *TAG = "DOG_TRACKER_BLE";
#define WIFI_SSID "YourSSID"
#define WIFI_PASS "YourPassword"
#define SERVER_URL "http://your-server.com/upload"
#define BATTERY_CAPACITY 350.0f // mAh
#define LIGHT_SLEEP_TIMEOUT (5 * 60 * 1000000ULL) // 5 min in us (default)
#define DEEP_SLEEP_TIMEOUT (10 * 60 * 1000000ULL) // 10 min in us (default)
#define ACTIVITY_WINDOW 10 // Number of samples to assess activity state
#define EPILEPSY_THRESHOLD 1.5f // High accel magnitude for epilepsy detection

// UUIDs
#define COORD_SERVICE_UUID "180A"
#define RX_CHAR_UUID       "2A58"
#define ECHO_CHAR_UUID     "2A59"
#define DIST_CHAR_UUID     "2A5A"
#define INFO_CHAR_UUID     "2A5B"
#define SPEED_CHAR_UUID    "2A5C"
#define STEPS_CHAR_UUID    "2A5D"
#define ACTIVITY_CHAR_UUID "2A5E"
#define BATTERY_CHAR_UUID  "2A5F"
#define CONFIG_CHAR_UUID   "2A60" // New characteristic for server config

static constexpr int kAdvIntervalMsDefault = 100;

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
static std::chrono::steady_clock::time_point prevTime, lastActivityTime;
static std::string dogBreed = "";
static int dogAge = 0;
static float dogSize = 0.0f, stepSize = 0.0f;
static bool isBleActive = false, isWifiActive = false;
static uint64_t lastSensorSample = 0, lastWifiUpload = 0;
static float batteryRemaining = BATTERY_CAPACITY; // mAh
static int epilepsyEvents = 0;

// Dynamic parameters
enum ActivityState { LOW, MEDIUM, HIGH };
static ActivityState activityState = MEDIUM;
static int sensorSampleInterval = 10000; // ms (default 10s)
static int sensorActiveTime = 100; // ms (default 0.1s)
static int wifiActiveTime = 10000; // ms (default 10s)
static uint64_t wifiUploadInterval = 24 * 3600 * 1000; // ms (default 24 hr)
static int bleConnIntervalMin = kAdvIntervalMsDefault; // ms
static int bleConnIntervalMax = kAdvIntervalMsDefault; // ms
static uint64_t lightSleepTimeout = LIGHT_SLEEP_TIMEOUT; // us
static uint64_t deepSleepTimeout = DEEP_SLEEP_TIMEOUT; // us

NimBLECharacteristic *activityChar = nullptr, *batteryChar = nullptr, *configChar = nullptr;

// SPIFFS file
#define LOG_FILE "/spiffs/log.txt"

// Function prototypes
void wifi_init();
void wifi_connect();
void wifi_upload_logs();
void spiffs_init();
void log_data(const char *data);
void enter_light_sleep();
void enter_deep_sleep();
float estimate_battery_life();
void adjustParameters();

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

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
        ESP_LOGI(TAG, "Client connected");
        isBleActive = true;
        lastActivityTime = std::chrono::steady_clock::now();
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
        ESP_LOGI(TAG, "Client disconnected, restarting advertising");
        isBleActive = false;
        NimBLEDevice::getAdvertising()->start();
        lastActivityTime = std::chrono::steady_clock::now();
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
        log_data(("DogInfo: " + dogBreed + "," + std::to_string(dogAge) + "," +
                  std::to_string(dogSize)).c_str());
    }
};

class ConfigCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChr, NimBLEConnInfo&) override {
        std::string raw = pChr->getValue();
        if (raw.empty()) return;

        raw.erase(std::remove_if(raw.begin(), raw.end(), ::isspace), raw.end());
        int sampleInt, wifiTime, wifiIntDays, bleMin, bleMax;
        if (std::sscanf(raw.c_str(), "%d,%d,%d,%d,%d", &sampleInt, &wifiTime, &wifiIntDays, &bleMin, &bleMax) != 5) return;

        sensorSampleInterval = sampleInt;
        wifiActiveTime = wifiTime;
        wifiUploadInterval = wifiIntDays * 24 * 3600 * 1000;
        bleConnIntervalMin = bleMin;
        bleConnIntervalMax = bleMax;

        NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
        adv->setMinInterval(bleConnIntervalMin / 0.625);
        adv->setMaxInterval(bleConnIntervalMax / 0.625);
        adv->start();

        ESP_LOGI(TAG, "Config updated: Sample=%d ms, WiFiTime=%d ms, WiFiInt=%d days, BLEMin=%d ms, BLEMax=%d ms",
                 sensorSampleInterval, wifiActiveTime, wifiIntDays, bleConnIntervalMin, bleConnIntervalMax);
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
        log_data(("GPS: " + std::to_string(lat) + "," + std::to_string(lon) + "," +
                  std::to_string(dist) + "," + std::to_string(speed) + "," +
                  std::to_string(steps)).c_str());

        prevLat = lat; prevLon = lon; prevTime = now;
        lastActivityTime = now;
    }
};

float get_moving_average() {
    float sum = 0;
    for (int i = 0; i < SAMPLE_WINDOW; ++i) sum += acc_magnitude_buffer[i];
    return sum / SAMPLE_WINDOW;
}

const char* detect_activity(float net_magnitude) {
    if (net_magnitude < 0.1f) return "Resting";
    else if (net_magnitude < 0.5f) return "Walking";
    else {
        if (net_magnitude > EPILEPSY_THRESHOLD) epilepsyEvents++;
        return "Running";
    }
}

void adjustParameters() {
    static int activityCounts[3] = {0}; // Resting, Walking, Running
    static int sampleCount = 0;

    // Update activity counts
    const char* activity = detect_activity(get_moving_average());
    if (strcmp(activity, "Resting") == 0) activityCounts[0]++;
    else if (strcmp(activity, "Walking") == 0) activityCounts[1]++;
    else activityCounts[2]++;
    sampleCount++;

    if (sampleCount >= ACTIVITY_WINDOW) {
        // Determine dominant activity
        if (activityCounts[2] >= ACTIVITY_WINDOW * 0.6) { // 60% Running
            activityState = HIGH;
            sensorSampleInterval = 5000; // 5s
            sensorActiveTime = 100; // 0.1s (2% duty cycle)
            wifiActiveTime = 15000; // 15s
            wifiUploadInterval = 12 * 3600 * 1000; // 12 hr
            bleConnIntervalMin = 50; // 50ms
            bleConnIntervalMax = 50;
            lightSleepTimeout = 10 * 60 * 1000000ULL; // 10 min
            deepSleepTimeout = 15 * 60 * 1000000ULL; // 15 min
        } else if (activityCounts[1] >= ACTIVITY_WINDOW * 0.5) { // 50% Walking
            activityState = MEDIUM;
            sensorSampleInterval = 10000; // 10s
            sensorActiveTime = 100; // 0.1s (1% duty cycle)
            wifiActiveTime = 10000; // 10s
            wifiUploadInterval = 24 * 3600 * 1000; // 24 hr
            bleConnIntervalMin = 100;
            bleConnIntervalMax = 100;
            lightSleepTimeout = LIGHT_SLEEP_TIMEOUT;
            deepSleepTimeout = DEEP_SLEEP_TIMEOUT;
        } else { // Mostly Resting
            activityState = LOW;
            sensorSampleInterval = 20000; // 20s
            sensorActiveTime = 100; // 0.1s (0.5% duty cycle)
            wifiActiveTime = 5000; // 5s
            wifiUploadInterval = 48 * 3600 * 1000; // 48 hr
            bleConnIntervalMin = 500; // 500ms
            bleConnIntervalMax = 500;
            lightSleepTimeout = 2 * 60 * 1000000ULL; // 2 min
            deepSleepTimeout = 5 * 60 * 1000000ULL; // 5 min
        }

        // Update BLE advertising
        NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
        adv->setMinInterval(bleConnIntervalMin / 0.625);
        adv->setMaxInterval(bleConnIntervalMax / 0.625);
        if (!isBleActive) adv->start();

        ESP_LOGI(TAG, "Adjusted: State=%d, Sample=%d ms, WiFiTime=%d ms, WiFiInt=%lld ms, BLEMin=%d ms",
                 activityState, sensorSampleInterval, wifiActiveTime, wifiUploadInterval, bleConnIntervalMin);

        // Reset counters
        activityCounts[0] = activityCounts[1] = activityCounts[2] = 0;
        sampleCount = 0;
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
    qmi.configGyroscope(SensorQMI8658::GYR_RANGE_64DPS, SensorQMI8658::GYR_ODR_896_8Hz,
                        SensorQMI8658::LPF_MODE_3, true);
    qmi.enableAccelerometer();
    qmi.enableGyroscope();
}

void spiffs_init() {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_vfs_spiffs_register(&conf);
    ESP_LOGI(TAG, "SPIFFS initialized");
}

void log_data(const char *data) {
    FILE *f = fopen(LOG_FILE, "a");
    if (f) {
        fprintf(f, "%s\n", data);
        fclose(f);
    } else {
        ESP_LOGE(TAG, "Failed to open log file");
    }
}

void wifi_init() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
}

void wifi_connect() {
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_connect();
    isWifiActive = true;
    ESP_LOGI(TAG, "Wi-Fi connecting...");
}

void wifi_upload_logs() {
    FILE *f = fopen(LOG_FILE, "r");
    if (!f) {
        ESP_LOGE(TAG, "No logs to upload");
        return;
    }

    char line[256];
    esp_http_client_config_t config = {
        .url = SERVER_URL,
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    while (fgets(line, sizeof(line), f)) {
        esp_http_client_set_post_field(client, line, strlen(line));
        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Log uploaded: %s", line);
        }
    }

    fclose(f);
    remove(LOG_FILE);
    esp_http_client_cleanup(client);
    esp_wifi_disconnect();
    esp_wifi_stop();
    isWifiActive = false;
    ESP_LOGI(TAG, "Wi-Fi upload complete");
}

float estimate_battery_life() {
    float dailyConsumption;
    switch (activityState) {
        case HIGH: dailyConsumption = 3.725f; break;
        case MEDIUM: dailyConsumption = 3.06f; break;
        case LOW: dailyConsumption = 1.54f; break;
        default: dailyConsumption = 3.06f;
    }
    return batteryRemaining / dailyConsumption; // Days
}

void enter_light_sleep() {
    esp_sleep_enable_timer_wakeup(lightSleepTimeout);
    ESP_LOGI(TAG, "Entering light sleep");
    esp_light_sleep_start();
    batteryRemaining -= 0.05f * (lightSleepTimeout / (3600.0f * 1000000.0f)); // Adjust for dynamic duration
}

void enter_deep_sleep() {
    esp_sleep_enable_timer_wakeup(deepSleepTimeout);
    ESP_LOGI(TAG, "Entering deep sleep");
    esp_deep_sleep_start();
    batteryRemaining -= 0.02f * (deepSleepTimeout / (3600.0f * 1000000.0f)); // Adjust for dynamic duration
}

void read_sensor_data(void* arg) {
    while (1) {
        uint64_t now = esp_timer_get_time() / 1000; // ms
        if (now - lastSensorSample >= sensorSampleInterval) {
            if (qmi.getDataReady()) {
                if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
                    float raw_mag = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
                    float net_mag = fabs(raw_mag - 1.0f);
                    acc_magnitude_buffer[buffer_index++] = net_mag;
                    if (buffer_index >= SAMPLE_WINDOW) buffer_index = 0;

                    float avg_mag = get_moving_average();
                    const char* activity = detect_activity(avg_mag);
                    ESP_LOGI(TAG, "Activity: %s, Accel: %.2f %.2f %.2f", activity, acc.x, acc.y, acc.z);
                    log_data(("Activity: " + std::string(activity) + ",Accel:" +
                              std::to_string(acc.x) + "," + std::to_string(acc.y) + "," +
                              std::to_string(acc.z)).c_str());

                    if (activityChar) {
                        activityChar->setValue(activity);
                        activityChar->notify();
                    }

                    // Adjust parameters based on activity
                    adjustParameters();

                    // Update battery
                    float dutyCycle = sensorActiveTime / (float)sensorSampleInterval;
                    batteryRemaining -= 10.0f * dutyCycle * (sensorActiveTime / 3600000.0f); // 10 mA base
                    if (epilepsyEvents > 0) {
                        batteryRemaining -= 15.0f * (EPILEPSY_EVENT_DURATION / 3600000.0f);
                        epilepsyEvents = 0;
                    }

                    // Update battery status
                    char buf[32];
                    std::snprintf(buf, sizeof(buf), "%.1f days", estimate_battery_life());
                    if (batteryChar) {
                        batteryChar->setValue(buf);
                        batteryChar->notify();
                    }
                }
            }
            lastSensorSample = now;
            lastActivityTime = std::chrono::steady_clock::now();
        }

        // Wi-Fi upload
        if (now - lastWifiUpload >= wifiUploadInterval) {
            wifi_connect();
            vTaskDelay(wifiActiveTime / portTICK_PERIOD_MS);
            wifi_upload_logs();
            lastWifiUpload = now;
            batteryRemaining -= 70.0f * (wifiActiveTime / 3600000.0f); // 70 mA
        }

        // Sleep mode logic
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - lastActivityTime).count();
        if (!isBleActive && !isWifiActive && elapsed >= lightSleepTimeout) {
            if (elapsed >= deepSleepTimeout) {
                enter_deep_sleep();
            } else {
                enter_light_sleep();
            }
        }

        vTaskDelay(sensorActiveTime / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    spiffs_init();
    wifi_init();

    NimBLEDevice::init("MJ");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    NimBLEDevice::setSecurityAuth(false, false, true);

    NimBLEServer *pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService *pSvc = pServer->createService(COORD_SERVICE_UUID);

    auto *rxChar = pSvc->createCharacteristic(RX_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
    auto *echoChar = pSvc->createCharacteristic(ECHO_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    auto *distChar = pSvc->createCharacteristic(DIST_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    auto *speedChar = pSvc->createCharacteristic(SPEED_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    auto *stepsChar = pSvc->createCharacteristic(STEPS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    auto *infoChar = pSvc->createCharacteristic(INFO_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
    activityChar = pSvc->createCharacteristic(ACTIVITY_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    batteryChar = pSvc->createCharacteristic(BATTERY_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    configChar = pSvc->createCharacteristic(CONFIG_CHAR_UUID, NIMBLE_PROPERTY::WRITE);

    infoChar->setCallbacks(new InfoCallbacks());
    rxChar->setCallbacks(new RxCallbacks(echoChar, distChar, speedChar, stepsChar));
    configChar->setCallbacks(new ConfigCallbacks());

    pSvc->start();

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->setAppearance(0x03C0);
    adv->setMinInterval(bleConnIntervalMin / 0.625);
    adv->setMaxInterval(bleConnIntervalMax / 0.625);
    adv->addServiceUUID(COORD_SERVICE_UUID);

    NimBLEAdvertisementData advData;
    advData.setCompleteServices(NimBLEUUID(COORD_SERVICE_UUID));
    advData.setAppearance(0x03C0);
    adv->setAdvertisementData(advData);

    NimBLEAdvertisementData scanRespData;
    scanRespData.setName("MJ");
    adv->setScanResponseData(scanRespData);

    adv->start();
    ESP_LOGI(TAG, "BLE started with name MJ");

    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f days", estimate_battery_life());
    batteryChar->setValue(buf);
    batteryChar->notify();

    setup_sensor();
    lastActivityTime = std::chrono::steady_clock::now();
    xTaskCreate(read_sensor_data, "sensor_read_task", 4096, NULL, 10, NULL);
}
