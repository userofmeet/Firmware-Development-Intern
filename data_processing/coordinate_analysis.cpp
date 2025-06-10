#include <cmath>
#include <cstring>
#include <string>
#include <algorithm>
#include "esp_log.h"
#include "nvs_flash.h"
#include "NimBLEDevice.h"
#include <chrono>

static const char *TAG = "DOG_TRACKER_BLE";

#define COORD_SERVICE_UUID "180A"
#define RX_CHAR_UUID "2A58" // The GPS coordinates are written here in the /* lattitude,longitude */ format
#define ECHO_CHAR_UUID "2A59"
#define DIST_CHAR_UUID "2A5A"
#define INFO_CHAR_UUID "2A5B"  // Enter BREED, AGE, SIZE in the /* breed,age,size */ format

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
static std::chrono::steady_clock::time_point prevTime;

static std::string dogBreed = "";
static int dogAge = 0;
static float dogSize = 0.0f; 
static float stepSize = 0.0f;
float estimateStepSize(const std::string &breed, int age, float size) {
    float baseStep = 0.0f;

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
    else if (breed == "Bulldog") baseStep = 0.45f;
    else baseStep = 0.60f; 
    if (size > 0.10f) {
        baseStep *= size / 0.5f; // assuming 0.5m is average size reference
    }

    // Adjust based on age (puppy vs adult vs senior)
    if (age < 1) baseStep *= 0.7f;    // puppies smaller steps
    else if (age > 8) baseStep *= 0.85f; // seniors slower/shorter steps

    return baseStep;
}

class ServerCallbacks : public NimBLEServerCallbacks {
public:
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        ESP_LOGI(TAG, "Client connected");
    }
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        ESP_LOGI(TAG, "Client disconnected, restarting advertising");
        NimBLEDevice::getAdvertising()->start();
    }
};

class InfoCallbacks : public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic *pChr, NimBLEConnInfo&) override {
        std::string raw = pChr->getValue();
        if (raw.empty()) return;

        // Expected format: "breed,age,size"
        // e.g. "Labrador,3,0.55"
        raw.erase(std::remove_if(raw.begin(), raw.end(), ::isspace), raw.end());

        char breedBuf[32] = {0};
        int age = 0;
        float size = 0.0f;

        int ret = std::sscanf(raw.c_str(), "%31[^,],%d,%f", breedBuf, &age, &size);
        if (ret != 3) {
            ESP_LOGW(TAG, "Bad dog info format: \"%s\"", raw.c_str());
            return;
        }

        dogBreed = std::string(breedBuf);
        dogAge = age;
        dogSize = size;
        stepSize = estimateStepSize(dogBreed, dogAge, dogSize);

        ESP_LOGI(TAG, "Dog info set - Breed: %s, Age: %d, Size: %.2f m, StepSize: %.2f m",
                 dogBreed.c_str(), dogAge, dogSize, stepSize);
    }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
    NimBLECharacteristic *echoChar_;
    NimBLECharacteristic *distChar_;
    NimBLECharacteristic *speedChar_;
    NimBLECharacteristic *stepsChar_;
public:
    RxCallbacks(NimBLECharacteristic *echoC, NimBLECharacteristic *distC, NimBLECharacteristic *speedC, NimBLECharacteristic *stepsC)
        : echoChar_(echoC), distChar_(distC), speedChar_(speedC), stepsChar_(stepsC) {}

    void onWrite(NimBLECharacteristic *pChr, NimBLEConnInfo&) override {
        std::string raw = pChr->getValue();
        if (raw.empty()) return;

        // Remove spaces so format like "lat , lon" is accepted
        raw.erase(std::remove_if(raw.begin(), raw.end(), ::isspace), raw.end());

        float lat, lon;
        if (std::sscanf(raw.c_str(), "%f,%f", &lat, &lon) != 2) {
            ESP_LOGW(TAG, "Bad coordinate format: \"%s\"", raw.c_str());
            return;
        }

        echoChar_->setValue(raw);
        echoChar_->notify();

        ESP_LOGI(TAG, "Received coordinates: %f, %f", lat, lon);

        float dist = 0.0f;
        float speed = 0.0f;
        int steps = 0;

        auto now = std::chrono::steady_clock::now();

        if (!std::isnan(prevLat) && !std::isnan(prevLon)) {
            dist = haversine_m(prevLat, prevLon, lat, lon);

            // Calculate time diff in seconds
            float timeSec = std::chrono::duration<float>(now - prevTime).count();
            if (timeSec > 0.001f) {
                speed = dist / timeSec;
            }

            if (stepSize > 0) {
                steps = static_cast<int>(dist / stepSize);
            }
        }

        // Distance
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.2f", dist);
        distChar_->setValue(buf);
        distChar_->notify();
        ESP_LOGI(TAG, "Distance: %.2f m", dist);

        // Speed
        std::snprintf(buf, sizeof(buf), "%.2f", speed);
        speedChar_->setValue(buf);
        speedChar_->notify();
        ESP_LOGI(TAG, "Speed: %.2f m/s", speed);

        // Steps
        std::snprintf(buf, sizeof(buf), "%d", steps);
        stepsChar_->setValue(buf);
        stepsChar_->notify();
        ESP_LOGI(TAG, "Steps: %d", steps);  // Now always printed, even if 0

        prevLat = lat;
        prevLon = lon;
        prevTime = now;
    }
};

extern "C" void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    NimBLEDevice::init("ESP-DogTracker");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    NimBLEServer *pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService *pSvc = pServer->createService(COORD_SERVICE_UUID);

    // Characteristic to receive coordinates
    auto *rxChar = pSvc->createCharacteristic(RX_CHAR_UUID, NIMBLE_PROPERTY::WRITE);

    // Characteristic to echo back last coordinates
    auto *echoChar = pSvc->createCharacteristic(
        ECHO_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // Characteristic to send distance in meters
    auto *distChar = pSvc->createCharacteristic(
        DIST_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // Characteristic to send speed in m/s
    auto *speedChar = pSvc->createCharacteristic(
        "2A5C",  // Custom UUID for speed
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // Characteristic to send steps count
    auto *stepsChar = pSvc->createCharacteristic(
        "2A5D",  // Custom UUID for steps
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // Characteristic to receive dog info (breed, age, size)
    auto *infoChar = pSvc->createCharacteristic(
        INFO_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE);

    infoChar->setCallbacks(new InfoCallbacks());

    rxChar->setCallbacks(new RxCallbacks(echoChar, distChar, speedChar, stepsChar));

    echoChar->createDescriptor("2901")->setValue("Echo last coordinate");
    distChar->createDescriptor("2901")->setValue("Distance (m)");
    speedChar->createDescriptor("2901")->setValue("Speed (m/s)");
    stepsChar->createDescriptor("2901")->setValue("Steps Count");
    infoChar->createDescriptor("2901")->setValue("Dog Info (breed,age,size)");

    pSvc->start();

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->setAppearance(0x03C0);
    adv->setMinInterval(kAdvIntervalMs / 0.625);
    adv->setMaxInterval(kAdvIntervalMs / 0.625);
    adv->addServiceUUID(COORD_SERVICE_UUID);
    adv->start();

    ESP_LOGI(TAG, "BLE started, awaiting dog info and coordinates...");
}
