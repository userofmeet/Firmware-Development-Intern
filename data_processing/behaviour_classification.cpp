#include <cstring>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "SensorLib.h"
#include "SensorQMI8658.hpp"

#define I2C_MASTER_SCL     14
#define I2C_MASTER_SDA     15
#define I2C_MASTER_NUM     I2C_NUM_0
#define QMI8658_ADDRESS    0x6B

static const char *TAG = "DOG_TRACKER";

SensorQMI8658 qmi;
IMUdata acc, gyr;

#define SAMPLE_WINDOW 10
float acc_magnitude_buffer[SAMPLE_WINDOW];
int buffer_index = 0;

int step_count = 0;
float distance_m = 0.0f;
float avg_step_length_m = 0.6f;  // Adjust per breed
uint32_t previous_step_time_ms = 0;
float speed_mps = 0.0f;

float get_moving_average() {
    float sum = 0;
    for (int i = 0; i < SAMPLE_WINDOW; ++i)
        sum += acc_magnitude_buffer[i];
    return sum / SAMPLE_WINDOW;
}

const char* detect_activity(float net_magnitude) {
    if (net_magnitude < 0.1f) return "Resting";
    else if (net_magnitude < 0.5f) return "Walking";
    else return "Running";
}

bool detect_step(float net_magnitude) {
    static bool was_below_threshold = true;
    const float step_threshold = 0.5f;  // tuned after gravity removal
    if (net_magnitude > step_threshold && was_below_threshold) {
        was_below_threshold = false;
        return true;
    } else if (net_magnitude < step_threshold) {
        was_below_threshold = true;
    }
    return false;
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
        ESP_LOGE(TAG, "Failed to initialize QMI8658.");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Device ID: %x", qmi.getChipID());

    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_0, true);
    qmi.configGyroscope(SensorQMI8658::GYR_RANGE_64DPS, SensorQMI8658::GYR_ODR_896_8Hz, SensorQMI8658::LPF_MODE_3, true);

    qmi.enableAccelerometer();
    qmi.enableGyroscope();

    ESP_LOGI(TAG, "Sensor Initialized");
}

void read_sensor_data(void* arg) {
    while (1) {
        if (qmi.getDataReady()) {
            if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
                // Calculate raw magnitude
                float raw_magnitude = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
                float net_magnitude = fabs(raw_magnitude - 1.0f);  // Remove gravity (~1g)

                acc_magnitude_buffer[buffer_index++] = net_magnitude;
                if (buffer_index >= SAMPLE_WINDOW) buffer_index = 0;

                float avg_magnitude = get_moving_average();
                const char* activity = detect_activity(avg_magnitude);

                uint32_t current_time = qmi.getTimestamp();

                if (detect_step(avg_magnitude)) {
                    step_count++;
                    distance_m = step_count * avg_step_length_m;

                    float delta_time_s = (current_time - previous_step_time_ms) / 1000.0f;
                    previous_step_time_ms = current_time;

                    if (delta_time_s > 0) {
                        speed_mps = avg_step_length_m / delta_time_s;
                    } else {
                        speed_mps = 0.0f;
                    }
                }

                ESP_LOGI(TAG, "Activity: %s | Steps: %d | Distance: %.2f m | Speed: %.2f m/s", activity, step_count, distance_m, speed_mps);
                ESP_LOGI(TAG, "Accel: %.2f, %.2f, %.2f", acc.x, acc.y, acc.z);
            }
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);  // 5Hz
    }
}

extern "C" void app_main() {
    setup_sensor();
    xTaskCreate(read_sensor_data, "sensor_read_task", 4096, NULL, 10, NULL);
}
