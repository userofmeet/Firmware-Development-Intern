#include <cmath>
#include <cstring>
#include <string>

#include "esp_log.h"
#include "nvs_flash.h"
#include "NimBLEDevice.h"
#include <algorithm>
static const char *TAG = "COORD_BLE";

#define COORD_SERVICE_UUID "180A"
#define RX_CHAR_UUID "2A58"
#define ECHO_CHAR_UUID "2A59"
#define DIST_CHAR_UUID "2A5A"

static constexpr bool kCalcFromHome = false;
static constexpr float kHomeLat = 23.0338;
static constexpr float kHomeLon = 72.5850;
static constexpr bool kEchoOverUART = true;
static constexpr int kAdvIntervalMs = 100;

static float deg2rad(float deg) {
    return deg * static_cast<float>(M_PI) / 180.0f;
}

static float haversine_m(float lat1, float lon1, float lat2, float lon2) {
    const float R = 6371000.0f;
    float dLat = deg2rad(lat2 - lat1);
    float dLon = deg2rad(lon2 - lon1);
    float a = std::pow(std::sin(dLat / 2), 2) +
              std::cos(deg2rad(lat1)) * std::cos(deg2rad(lat2)) *
              std::pow(std::sin(dLon / 2), 2);
    float c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return R * c;
}

static float prevLat = NAN;
static float prevLon = NAN;

class RxCallbacks : public NimBLECharacteristicCallbacks {
    NimBLECharacteristic *echoChar_;
    NimBLECharacteristic *distChar_;
public:
    RxCallbacks(NimBLECharacteristic *echoC, NimBLECharacteristic *distC)
        : echoChar_(echoC), distChar_(distC) {}
    void onWrite(NimBLECharacteristic *pChr, NimBLEConnInfo&) override {
        std::string raw = pChr->getValue();
        if (raw.empty()) return;

        // Optional: remove spaces to allow "lat , lon" too
        raw.erase(std::remove_if(raw.begin(), raw.end(), ::isspace), raw.end());

        float lat, lon;
        if (std::sscanf(raw.c_str(), "%f,%f", &lat, &lon) != 2) {
            ESP_LOGW(TAG, "Bad format: \"%s\"", raw.c_str());
            return;
        }

        // Echo back the cleaned raw string
        echoChar_->setValue(raw);
        echoChar_->notify();

        if (kEchoOverUART) {
            ESP_LOGI(TAG, "RX %f , %f", lat, lon);
        }

        float dist = NAN;
        if (!std::isnan(prevLat) && !std::isnan(prevLon)) {
            dist = haversine_m(prevLat, prevLon, lat, lon);
        }

        if (!std::isnan(dist)) {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.2f", dist);
            distChar_->setValue(buf);
            distChar_->notify();

            if (kEchoOverUART) {
                ESP_LOGI(TAG, "Distance: %.2f m", dist);
            }
        }

        // Save current as previous for next distance calculation
        prevLat = lat;
        prevLon = lon;
    }
};

extern "C" void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    NimBLEDevice::init("ESP-Coord");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    NimBLEServer *pServer = NimBLEDevice::createServer();

    NimBLEService *pSvc = pServer->createService(COORD_SERVICE_UUID);

    auto *rxChar = pSvc->createCharacteristic(RX_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
    auto *echoChar = pSvc->createCharacteristic(
        ECHO_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    auto *distChar = pSvc->createCharacteristic(
        DIST_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    rxChar->setCallbacks(new RxCallbacks(echoChar, distChar));

    echoChar->createDescriptor("2901")->setValue("Echo of last point");
    distChar->createDescriptor("2901")->setValue("Distance (m)");

    pSvc->start();

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->setAppearance(0x03C0);
    adv->setMinInterval(kAdvIntervalMs / 0.625);
    adv->setMaxInterval(kAdvIntervalMs / 0.625);
    adv->addServiceUUID(COORD_SERVICE_UUID);
    adv->start();

    ESP_LOGI(TAG, "BLE started, awaiting coordinates …");
}
