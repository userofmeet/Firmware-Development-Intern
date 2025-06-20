#include <cmath>
#include <cstring>
#include <string>
#include <algorithm>
#include <chrono>

#include "esp_log.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "NimBLEDevice.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs.h"
#include "mqtt_client.h"
#include "SensorLib.h"
#include "SensorQMI8658.hpp"

static const char *TAG = "DOG_TRACKER_BLE";

// UUIDs
#define COORD_SERVICE_UUID "180A"
#define RX_CHAR_UUID       "2A58"
#define ECHO_CHAR_UUID     "2A59"

// I2C + Sensor
#define I2C_MASTER_SCL
