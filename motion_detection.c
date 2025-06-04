#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 14
#define I2C_MASTER_SDA_IO 15
#define I2C_MASTER_FREQ_HZ 100000

#define QMI8658C_I2C_ADDR 0x6B
#define WHO_AM_I_REG 0x00
#define WHO_AM_I_VAL 0x05

#define CTRL1_REG 0x02
#define CTRL2_REG 0x03
#define CTRL3_REG 0x04
#define CTRL7_REG 0x08

#define ACC_X_L 0x35
#define GYR_X_L 0x3B

#define CALIBRATION_SAMPLES 100
#define MOTION_COUNT_THRESHOLD 5
#define REST_COUNT_THRESHOLD 5
#define NOISE_THRESHOLD 0.05f
#define LOG_INTERVAL 1// log every 10 stable loops
#define BASELINE_FALLBACK 1000.0f   
#define DYNAMIC_THRESHOLD_MULTIPLIER 1.3f 
static const char *TAG = "QMI8658C";

typedef enum {
    RESTING,
    MOVING
} MotionState;

static float gyro_offsets[3] = {0.0f, 0.0f, 0.0f};
static float accel_offsets[3] = {0.0f, 0.0f, 0.0f};
static float dynamic_threshold = NOISE_THRESHOLD;

#define LPF_ALPHA 0.05f
static float prev_magnitude = 0.0f;

void i2c_master_init() {
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, config.mode, 0, 0, 0));
}

esp_err_t i2c_write_register(uint8_t reg_addr, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMI8658C_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_read_register(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMI8658C_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMI8658C_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void qmi8658c_init() {
    uint8_t who_am_i;
    if (i2c_read_register(WHO_AM_I_REG, &who_am_i, 1) != ESP_OK || who_am_i != WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I check failed. Got: 0x%02X", who_am_i);
        return;
    }

    ESP_LOGI(TAG, "Device recognized: 0x%02X", who_am_i);

    i2c_write_register(CTRL7_REG, 0x03);
    i2c_write_register(CTRL2_REG, 0x11);
    i2c_write_register(CTRL3_REG, 0x11);
}

void calibrate_sensors() {
    uint8_t data[6];
    int16_t ax, ay, az, gx, gy, gz;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        if (i2c_read_register(ACC_X_L, data, 6) == ESP_OK) {
            ax = (int16_t)(data[1] << 8 | data[0]);
            ay = (int16_t)(data[3] << 8 | data[2]);
            az = (int16_t)(data[5] << 8 | data[4]);

            accel_offsets[0] += ax;
            accel_offsets[1] += ay;
            accel_offsets[2] += az;

            if (i2c_read_register(GYR_X_L, data, 6) == ESP_OK) {
                gx = (int16_t)(data[1] << 8 | data[0]);
                gy = (int16_t)(data[3] << 8 | data[2]);
                gz = (int16_t)(data[5] << 8 | data[4]);

                gyro_offsets[0] += gx;
                gyro_offsets[1] += gy;
                gyro_offsets[2] += gz;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    for (int i = 0; i < 3; i++) {
        accel_offsets[i] /= CALIBRATION_SAMPLES;
        gyro_offsets[i] /= CALIBRATION_SAMPLES;
    }

    ESP_LOGI(TAG, "Sensor calibration complete");
}


void read_sensor_data() {
    uint8_t data[6];
    int16_t ax, ay, az, gx, gy, gz;
    MotionState current_state = RESTING;
    MotionState previous_state = RESTING;
    int motion_count = 0, rest_count = 0;
    float baseline_magnitude = 0.0f;
    int baseline_samples = 200;
    int stable_state_counter = 0;

    // Calculate baseline magnitude
    for (int i = 0; i < baseline_samples; i++) {
        if (i2c_read_register(ACC_X_L, data, 6) == ESP_OK) {
            ax = (int16_t)(data[1] << 8 | data[0]) - accel_offsets[0];
            ay = (int16_t)(data[3] << 8 | data[2]) - accel_offsets[1];
            az = (int16_t)(data[5] << 8 | data[4]) - accel_offsets[2];

            if (i2c_read_register(GYR_X_L, data, 6) == ESP_OK) {
                gx = (int16_t)(data[1] << 8 | data[0]) - gyro_offsets[0];
                gy = (int16_t)(data[3] << 8 | data[2]) - gyro_offsets[1];
                gz = (int16_t)(data[5] << 8 | data[4]) - gyro_offsets[2];

                float raw_magnitude = ax*ax + ay*ay + az*az + gx*gx + gy*gy + gz*gz;
                baseline_magnitude += raw_magnitude;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    baseline_magnitude /= baseline_samples;
    if (baseline_magnitude <= 0.0f || isnan(baseline_magnitude)) {
        baseline_magnitude = BASELINE_FALLBACK;
        ESP_LOGW(TAG, "Baseline fallback used.");
    }

    dynamic_threshold = baseline_magnitude * DYNAMIC_THRESHOLD_MULTIPLIER;
    prev_magnitude = baseline_magnitude;

    ESP_LOGI(TAG, "Threshold initialized: %.2f", dynamic_threshold);

    while (1) {
        if (i2c_read_register(ACC_X_L, data, 6) == ESP_OK) {
            ax = (int16_t)(data[1] << 8 | data[0]) - accel_offsets[0];
            ay = (int16_t)(data[3] << 8 | data[2]) - accel_offsets[1];
            az = (int16_t)(data[5] << 8 | data[4]) - accel_offsets[2];

            if (i2c_read_register(GYR_X_L, data, 6) == ESP_OK) {
                gx = (int16_t)(data[1] << 8 | data[0]) - gyro_offsets[0];
                gy = (int16_t)(data[3] << 8 | data[2]) - gyro_offsets[1];
                gz = (int16_t)(data[5] << 8 | data[4]) - gyro_offsets[2];

                float raw_magnitude = ax*ax + ay*ay + az*az + gx*gx + gy*gy + gz*gz;
                float magnitude = LPF_ALPHA * raw_magnitude + (1.0f - LPF_ALPHA) * prev_magnitude;
                prev_magnitude = magnitude;

                // Ignore tiny noise — small deadzone around baseline
                if (fabsf(magnitude - baseline_magnitude) < (baseline_magnitude * 0.05f)) {
                    magnitude = baseline_magnitude;
                }

                if (magnitude > dynamic_threshold) {
                    motion_count++;
                    rest_count = 0;
                    if (motion_count > MOTION_COUNT_THRESHOLD) current_state = MOVING;
                } else {
                    rest_count++;
                    motion_count = 0;
                    if (rest_count > REST_COUNT_THRESHOLD) current_state = RESTING;
                }

                if (current_state == RESTING) {
                    // Smoothly adjust threshold toward baseline when resting
                    dynamic_threshold += (baseline_magnitude - dynamic_threshold) * 0.02f;
                }

                if (current_state != previous_state) {
                    stable_state_counter = 0;
                    previous_state = current_state;
                    ESP_LOGI(TAG, "Status: %s", current_state == MOVING ? "Moving" : "Resting");
                } else {
                    stable_state_counter++;
                    if (stable_state_counter % LOG_INTERVAL == 0) {
                        ESP_LOGI(TAG, "Status: %s", current_state == MOVING ? "Moving" : "Resting");
                    }
                }
            }
        } else {
            ESP_LOGE(TAG, "Failed to read sensor data");
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}


void app_main() {
    i2c_master_init();
    qmi8658c_init();
    calibrate_sensors();
    read_sensor_data();
}
