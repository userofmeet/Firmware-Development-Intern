#include <cmath>
#include <cstring>
#include <string>
#include <algorithm>
#include <vector>
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
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
#define SERVER_URL "http://192.168.1.100/upload"            // Enter your server address here
#define UPLOAD_INTERVAL_MS (12ULL * 60 * 60 * 1000)

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
#define MONITOR_WINDOW_MS (5 * 60 * 1000ULL)                        // BLE trigger every 5 minutes
#define BLE_ON_MS 10000                                             // 10 seconds to keep BLE advertising
#define RESTING_10_MIN_MS (5 * 60 * 1000ULL)                        // 5 minutes in milliseconds to trigger deep sleep

static const char *TAG = "DOG_TRACKER_BLE";

enum DogType { ADULT, STANDARD, ACTIVE };
struct DogProfile {
    DogType type;
    float thr_rest;    // net magnitude thresholds
    float thr_walk;
    int freq_rest;     // samples/sec
    int freq_walk;
    int freq_run;
};
DogProfile currentProfile;

SensorQMI8658 qmi;
IMUdata acc;

float acc_buffer[SAMPLE_WINDOW];
int buf_idx = 0;
bool buf_full = false;

NimBLEServer *pServer = nullptr;
NimBLEAdvertising *adv = nullptr;
NimBLECharacteristic* activityChar = nullptr;
NimBLECharacteristic* echoChar = nullptr;

int totalSamples = 0, activeSamples = 0;
uint64_t windowStart = 0, restingStart = 0;
const char* currAct = "Resting";
const char* pendingAct = nullptr;
int actCount = 0;

float offset_x=0,offset_y=0,offset_z=0,scale_f=1;
bool firstBufFill = false;

// ------------ UTILITIES & NVS ------------
void log_event(const std::string& line) {
    FILE* f = fopen(LOG_FILE_PATH, "a");
    if (!f) { ESP_LOGE(TAG,"Failed opening log"); return; }
    fprintf(f, "%llu: %s\n", esp_timer_get_time()/1000ULL, line.c_str());
    fclose(f);
}

float moving_avg() {
    float s=0;
    for(int i=0;i<SAMPLE_WINDOW;i++) s+=acc_buffer[i];
    return s/SAMPLE_WINDOW;
}

void start_ble() {
    if(!adv->isAdvertising()){
        adv->start();
        ESP_LOGI(TAG,"BLE ON");
        log_event("BLE ON");
    }
}

void stop_ble(){
    if(adv->isAdvertising()){
        adv->stop();
        ESP_LOGI(TAG,"BLE OFF");
        log_event("BLE OFF");
    }
}

// NVS save/load profile
void save_profile() {
    nvs_handle h;
    nvs_open("profile", NVS_READWRITE, &h);
    nvs_set_i32(h, "type", currentProfile.type);
    nvs_set_blob(h, "prof", &currentProfile, sizeof(currentProfile));
    nvs_set_u64(h, "last_update", esp_timer_get_time() / 1000000ULL); // store in seconds
    nvs_commit(h);
    nvs_close(h);
}

bool load_profile() {
    nvs_handle h;
    if (nvs_open("profile", NVS_READONLY, &h) != ESP_OK) return false;
    int32_t t;
    if (nvs_get_i32(h, "type", &t) != ESP_OK) { nvs_close(h); return false; }
    if (t < 0 || t > 2) { nvs_close(h); return false; }

    size_t sz = sizeof(currentProfile);
    if (nvs_get_blob(h, "prof", &currentProfile, &sz) != ESP_OK) { nvs_close(h); return false; }

    uint64_t last_update = 0;
    nvs_get_u64(h, "last_update", &last_update);
    uint64_t now_sec = esp_timer_get_time() / 1000000ULL;
    ESP_LOGI(TAG, "Loaded profile type %d, age = %llu days", currentProfile.type, (now_sec - last_update) / (24 * 60 * 60));
    
    nvs_close(h);
    return true;
}


// ------------ PROFILING LOGIC ------------
DogProfile build_profile(DogType t){
    DogProfile p;
    p.type=t;
    if(t==ADULT){ p.thr_rest=0.03; p.thr_walk=0.12; p.freq_rest=1; p.freq_walk=2; p.freq_run=4; }
    else if(t==STANDARD){ p.thr_rest=0.1; p.thr_walk=0.5; p.freq_rest=0.1; p.freq_walk=1; p.freq_run=2; }
    else { p.thr_rest=0.2; p.thr_walk=0.8; p.freq_rest=0.2; p.freq_walk=2; p.freq_run=4; }
    return p;
}

DogType classify_profile(float avg, float var) {
    if(var<0.01) return ADULT;
    if(var<0.05) return STANDARD;
    return ACTIVE;
}

void profiler_task(void* arg) {
    ESP_LOGI(TAG,"Profiling start (2min)...");
    const int total = 2*60*5; // 5Hz samples for 120s
    std::vector<float> samples;
    for(int i=0;i<total;i++){
        if(qmi.getDataReady() && qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
            float mag = fabs(sqrt(acc.x*acc.x+acc.y*acc.y+acc.z*acc.z)-1.0f);
            samples.push_back(mag);
        }
        vTaskDelay(200/portTICK_PERIOD_MS);
    }
    float sum=0; for(auto &v:samples) sum+=v;
    float avg=sum/samples.size(), var=0;
    for(auto &v:samples) var += (v-avg)*(v-avg);
    var /= samples.size();
    DogType dt = classify_profile(avg,var);
    currentProfile = build_profile(dt);
    save_profile();
    ESP_LOGI(TAG,"Profile done: type=%d, avg=%.3f,var=%.3f",dt,avg,var);
    vTaskDelete(NULL);
}

// sensor and wakeup calibration
void calibrate_sensor(){
    float sx=0,sy=0,sz=0,sm=0; int cnt=0;
    for(int i=0;i<100;i++){
        if(qmi.getDataReady() && qmi.getAccelerometer(acc.x,acc.y,acc.z)){
            float m=sqrt(acc.x*acc.x+acc.y*acc.y+acc.z*acc.z);
            sx+=acc.x; sy+=acc.y; sz+=acc.z; sm+=m; cnt++;
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
    if(cnt){ offset_x=sx/cnt; offset_y=sy/cnt; offset_z=sz/cnt; scale_f=1/(sm/cnt); }
}

void auto_profile_update_task(void* arg) {
    while (1) {
        uint64_t now_sec = esp_timer_get_time() / 1000000ULL;
        nvs_handle h;
        uint64_t last_update = 0;
        if (nvs_open("profile", NVS_READONLY, &h) == ESP_OK) {
            nvs_get_u64(h, "last_update", &last_update);
            nvs_close(h);
        }
        uint64_t elapsed_days = (now_sec - last_update) / (24 * 60 * 60);
        if (elapsed_days >= 3) {
            ESP_LOGI(TAG, "Auto profile update triggered (%llu days)", elapsed_days);
            log_event("Auto profile update triggered");
            xTaskCreate(profiler_task, "profiler", 8192, nullptr, 8, nullptr);
        }
        vTaskDelay(6 * 60 * 60 * 1000 / portTICK_PERIOD_MS);  // Check every 6 hours
    }
}


void setup_motion_wakeup(){
    gpio_config_t io = {};
    io.intr_type = GPIO_INTR_DISABLE;
    io.mode = GPIO_MODE_INPUT;
    io.pin_bit_mask = 1ULL<<QMI8658_INT_PIN;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)QMI8658_INT_PIN,1);
}

// wifi and log upload
void init_wifi() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
}

bool connect_wifi() {
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to Wi-Fi...");
    ESP_ERROR_CHECK(esp_wifi_connect());

    // Wait for connection with timeout
    int retries = 0;
    while (retries < 10) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            ESP_LOGI(TAG, "Connected to SSID: %s", ap_info.ssid);
            return true;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        retries++;
    }

    ESP_LOGW(TAG, "Wi-Fi connect failed");
    return false;
}

void upload_logs() {
    FILE* f = fopen(LOG_FILE_PATH, "r");
    if (!f) {
        ESP_LOGW(TAG, "No log file found to upload");
        return;
    }

    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    rewind(f);

    if (file_size <= 0) {
        fclose(f);
        ESP_LOGI(TAG, "Log file is empty");
        return;
    }

    char* log_data = (char*)malloc(file_size + 1);
    if (!log_data) {
        fclose(f);
        ESP_LOGE(TAG, "Failed to allocate memory for log upload");
        return;
    }

    fread(log_data, 1, file_size, f);
    log_data[file_size] = '\0';
    fclose(f);

    esp_http_client_config_t config = {
        .url = SERVER_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "text/plain");
    esp_http_client_set_post_field(client, log_data, file_size);

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK && esp_http_client_get_status_code(client) == 200) {
        ESP_LOGI(TAG, "Log upload success, clearing log file");
        FILE* wf = fopen(LOG_FILE_PATH, "w");
        if (wf) fclose(wf); // truncate the file
    } else {
        ESP_LOGW(TAG, "Log upload failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    free(log_data);
}

void upload_task(void*arg){
    while(1){
        init_wifi();
        if(connect_wifi()) { upload_logs(); esp_wifi_disconnect(); }
        esp_wifi_stop();
        vTaskDelay(UPLOAD_INTERVAL_MS/portTICK_PERIOD_MS);
    }
}

// detect the activity based on the magnitude
const char* detect_activity(float m){
    if(m<currentProfile.thr_rest) return "Resting";
    if(m<currentProfile.thr_walk) return "Walking";
    return "Running";
}

void sensor_task(void*arg){
    calibrate_sensor();
    windowStart = esp_timer_get_time()/1000ULL;
    int interval_ms = 1000/currentProfile.freq_rest;
    while(1){
        if(qmi.getDataReady() && qmi.getAccelerometer(acc.x,acc.y,acc.z)){
            float m = fabs(sqrt(acc.x*acc.x+acc.y*acc.y+acc.z*acc.z)-1.0f);
            if(!firstBufFill){
                std::fill_n(acc_buffer, SAMPLE_WINDOW, m);
                firstBufFill = buf_full = true;
            } else {
                acc_buffer[buf_idx++] = m;
                if(buf_idx>=SAMPLE_WINDOW){ buf_idx=0; buf_full=true; }
            }
            if(buf_full){
                float avg = moving_avg();
                const char* act = detect_activity(avg);
                log_event(std::string("Act=")+act);
                // state switching...
                if(strcmp(act,currAct)!=0){
                    int req=3; // same thresholds
                    if(!strcmp(currAct,"Resting")&& !strcmp(act,"Walking")) req=2;
                    if(!strcmp(currAct,"Walking")&& !strcmp(act,"Running")) req=5;
                    if(!pendingAct||strcmp(act,pendingAct)){
                        pendingAct=act; actCount=1;
                    } else if(++actCount>=req){
                        currAct=pendingAct; pendingAct=nullptr; actCount=0;
                        if(!strcmp(currAct,"Resting")){
                            interval_ms = 1000/currentProfile.freq_rest;
                            restingStart = esp_timer_get_time()/1000ULL;
                        } else if(!strcmp(currAct,"Walking"))
                            interval_ms = 1000/currentProfile.freq_walk;
                        else
                            interval_ms = 1000/currentProfile.freq_run;
                    }
                }
                totalSamples++;
                if(strcmp(act,"Walking")==0 || strcmp(act,"Running")==0)
                    activeSamples++;
                if(activityChar && adv->isAdvertising()){
                    activityChar->setValue(act);
                    activityChar->notify();
                }
                uint64_t now = esp_timer_get_time()/1000ULL;
                if(now-windowStart >= MONITOR_WINDOW_MS){
                    float pct = (totalSamples?100.0f*activeSamples/totalSamples:0);
                    if(pct>=70.0f){ start_ble(); vTaskDelay(BLE_ON_MS/portTICK_PERIOD_MS); stop_ble(); restingStart=0; }
                    windowStart=now; totalSamples=activeSamples=0;
                }
                if(!strcmp(currAct,"Resting") && restingStart && (now-restingStart) > RESTING_10_MIN_MS){
                    log_event("DeepSleep");
                    setup_motion_wakeup(); esp_deep_sleep_start();
                }
            }
        }
        vTaskDelay(interval_ms/portTICK_PERIOD_MS);
    }
}

// BLE callback
class RxCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChr, NimBLEConnInfo&){
        std::string s = pChr->getValue();
        if(strcasecmp(s.c_str(),"RECALIBRATE")==0){
            xTaskCreate(profiler_task,"profiler",8192,nullptr,8,nullptr);
            ESP_LOGI(TAG,"Recalibration started");
            log_event("Recalibrate");
            echoChar->setValue("Recalibrating");
            echoChar->notify();
        } else {
            echoChar->setValue(s);
            echoChar->notify();
        }
    }
};

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
// ------------ APP MAIN ------------
extern "C" void app_main(){
    ESP_ERROR_CHECK(nvs_flash_init());
    esp_vfs_spiffs_conf_t sp = {"/spiffs", nullptr, 5, true};
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&sp));

    NimBLEDevice::init("MJ");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    NimBLEDevice::setSecurityAuth(false,false,true);
    pServer = NimBLEDevice::createServer();
    auto svc = pServer->createService(COORD_SERVICE_UUID);
    auto rx = svc->createCharacteristic(RX_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
    echoChar = svc->createCharacteristic(ECHO_CHAR_UUID, NIMBLE_PROPERTY::READ|NIMBLE_PROPERTY::NOTIFY);
    activityChar = svc->createCharacteristic(ACTIVITY_CHAR_UUID, NIMBLE_PROPERTY::READ|NIMBLE_PROPERTY::NOTIFY);
    rx->setCallbacks(new RxCallbacks());
    svc->start();
    adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(COORD_SERVICE_UUID);

    setup_sensor();
    if(!load_profile()) xTaskCreate(profiler_task,"profiler",8192,nullptr,8,nullptr);
    xTaskCreate(sensor_task,"sensor",8192,nullptr,10,nullptr);
    xTaskCreate(upload_task,"uploader",8192,nullptr,5,nullptr);
    xTaskCreate(auto_profile_update_task, "auto_prof", 4096, nullptr, 4, nullptr);

}
