#include <cstring>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "SensorLib.h"
#include "SensorQMI8658.hpp"
#include <inttypes.h>
#include "esp_timer.h"
// I2C Configuration
#define I2C_MASTER_SCL       14
#define I2C_MASTER_SDA       15
#define I2C_MASTER_NUM       I2C_NUM_0
#define QMI8658_ADDRESS      0x6B

// Motion threshold (tune this as needed)
#define MOTION_THRESHOLD     0.15f  // Sensitive and accurate for small changes

static const char *TAG = "QMI8658";

SensorQMI8658 qmi;
IMUdata acc, gyr;
IMUdata prev_acc = {0.0f, 0.0f, 0.0f};  // Previous accelerometer reading

//
// I2C Initialization
//
void i2c_master_init() {
    i2c_config_t conf;
    memset(&conf, 0, sizeof(i2c_config_t));  // Zero initialize all fields

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;

#if ESP_IDF_VERSION_MAJOR >= 5
    conf.clk_flags = 0;
#endif

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

//
// QMI8658 Sensor Setup
//
void setup_qmi8658_sensor() {
    i2c_master_init();

    if (!qmi.begin(I2C_MASTER_NUM, QMI8658_ADDRESS, I2C_MASTER_SDA, I2C_MASTER_SCL)) {
        ESP_LOGE(TAG, "QMI8658 not found. Check wiring.");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Device ID: 0x%X", qmi.getChipID());

    qmi.configAccelerometer(
        SensorQMI8658::ACC_RANGE_4G,
        SensorQMI8658::ACC_ODR_1000Hz,
        SensorQMI8658::LPF_MODE_0,
        true
    );

    qmi.configGyroscope(
        SensorQMI8658::GYR_RANGE_64DPS,
        SensorQMI8658::GYR_ODR_896_8Hz,
        SensorQMI8658::LPF_MODE_3,
        true
    );

    qmi.enableGyroscope();
    qmi.enableAccelerometer();

    ESP_LOGI(TAG, "Sensor initialized.");
}

//
// Sensor Data Task
//
void read_sensor_task(void *arg)
{
    constexpr float     GRAVITY         = 1.0f;
    constexpr float     MIN_PEAK_TH     = 0.30f;
    constexpr float     MAX_PEAK_TH     = 2.50f;
    constexpr float     ALPHA_HP        = 0.90f;
    constexpr float     TH_DECAY        = 0.01f;
    constexpr uint32_t  REFRACTORY_MS   = 180;

    float acc_lp_mag  = GRAVITY;
    float adaptive_th = 0.60f;

    uint32_t    last_step_time = 0;
    uint32_t    step_count     = 0;

    while (true)
    {
        if (!qmi.getDataReady()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (!qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
            ESP_LOGE(TAG, "Accel read error");
            continue;
        }

        /* 1. magnitude */
        float mag = sqrtf(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);

        /* 2. high-pass (gravity removal) */
        acc_lp_mag = ALPHA_HP * acc_lp_mag + (1.0f - ALPHA_HP) * mag;
        float lin_acc = mag - acc_lp_mag;

        /* 3. rectified signal */
        float sig = fabsf(lin_acc);

        /* 4. adaptive threshold */
        adaptive_th += TH_DECAY * (sig - adaptive_th);
        adaptive_th  = fminf(fmaxf(adaptive_th, MIN_PEAK_TH), MAX_PEAK_TH);

        /* 5. peak + refractory */
        uint32_t now = esp_log_timestamp();
        static float prev_sig = 0.0f;
        bool rising = (sig > prev_sig);
        prev_sig = sig;

        if (sig > adaptive_th && !rising && (now - last_step_time) > REFRACTORY_MS) {
            ++step_count;
            last_step_time = now;

            /* ---- LOG FIX: use %lu ---- */
            ESP_LOGI(TAG,
                     "[STEP] Count=%lu  Peak=%.2f  Th=%.2f",
                     (unsigned long)step_count, sig, adaptive_th);
        }

        /* optional debug */
        ESP_LOGD(TAG,
                 "Raw=%.3f  Lin=%.3f  Sig=%.3f  Th=%.3f  Steps=%lu",
                 mag, lin_acc, sig, adaptive_th,
                 (unsigned long)step_count);

        vTaskDelay(pdMS_TO_TICKS(20));     // 50 Hz loop
    }
}
//
// Main Entry
//
extern "C" void app_main() {
    setup_qmi8658_sensor();
    xTaskCreate(read_sensor_task, "read_sensor_task", 4096, NULL, 10, NULL);
}