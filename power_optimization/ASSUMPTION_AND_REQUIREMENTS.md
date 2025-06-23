# Finalized Requirements and Assumptions 
## 1. Hardware and Platform
```
Microcontroller: ESP32-S3
Battery: 3.7V, 350 mAh (1295 mWh)
Battery Life Goal: ~120 days for standard dogs; ~90 days for active dogs; ~150 days for adult dogs
Sensors: Accelerometer only (QMI8658, no GPS or other sensors)
Communication: BLE for echoing coordinates, Wi-Fi for uploading logs
Storage: 10 MB flash for offline logs (SPIFFS)
Framework: ESP-IDF (C/C++)
```
# 2. Activity Detection
```
Activities: Walking, running, resting (classified using accelerometer data).
Majority Activity: Activity occurring ≥70% in a 5-minute sliding window (30 samples at 10-second intervals).
Sampling: Check activity every 10 seconds to balance power and accuracy.
```
# 3. Logging Frequencies
Optimized for battery life and dog type:
```
Standard Dog (120-day target):
Walking: 1 Hz (1 sample/second, ~6 bytes accelerometer + 1 byte activity).
Running: 2 Hz (2 samples/second).
Resting: 0.1 Hz (1 sample every 10 seconds).
Light Sleep: 0.05 Hz (1 sample every 20 seconds).
Deep Sleep: 0.01 Hz (1 sample every 100 seconds).
```
```
Active Dog (~90 days):
Walking: 2 Hz.
Running: 4 Hz.
Resting: 0.2 Hz.
Light Sleep: 0.1 Hz.
Deep Sleep: 0.02 Hz.
```
```
Adult Dog (~150 days):
Walking: 0.5 Hz.
Running: 1 Hz.
Resting: 0.05 Hz.
Light Sleep: 0.025 Hz.
Deep Sleep: 0.005 Hz.
```
Data Logged:
```
Accelerometer (x, y, z, ~6 bytes).
Activity type (~1 byte: 0=rest, 1=walk, 2=run).
Coordinates (latitude, longitude, ~16 bytes, when echoed via BLE).
Timestamp (~4 bytes, seconds since boot).
Total Log Entry Size: ~27 bytes per entry.
```
# 5. BLE Coordinate Echoing
```
Function: Receive coordinates (latitude, longitude, decimal degrees) from the phone via BLE and echo them back.
Frequency: Echo coordinates every 10 seconds during a 1-minute BLE active period (6 transmissions per period).
Activation Interval: BLE activates for 1 minute every 10 minutes when the dog is outdoors (walking or running ≥70% in a 5-minute window).
Example: For 2 hours outdoors (120 minutes), BLE activates 12 times (at t=5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110 minutes), for a total of 12 minutes, sending 72 coordinate transmissions.
Deactivation: BLE turns off after each 1-minute period, restarting after 10 minutes if outdoor condition persists.
Power: BLE off when not active to save power (~5 mA when active, ~0.25 mA average over outdoor periods).
```
# 6. Wi-Fi Data Upload
```
Interval: Activate Wi-Fi every 12 hours (e.g., 12 AM, 12 PM, or 43200 seconds).
Retry Logic: Attempt connection 3 times per interval, with 30-second delays between retries.
Server: -
Protocol: -
```
``` 
Log File:
Stores accelerometer data, activity type, echoed coordinates, and timestamps in 10 MB SPIFFS.
Estimated size: ~50 KB/day for standard dog (1 Hz walk, 2 Hz run, 0.1 Hz rest, coordinates when echoed).
10 MB stores ~200 days of offline data.
```
```
Behavior:
Append logs to SPIFFS if Wi-Fi is unavailable.
Upload all accumulated logs on successful connection.
Clear file after successful upload.
Power: Wi-Fi off outside upload intervals (~100 mA for 10 seconds per interval, ~0.01 mA average).
```
# 7. Outdoor Detection
```
Logic: Dog is outdoors if walking or running is ≥70% in a 5-minute sliding window (30 samples at 10-second intervals).
Action: Activate BLE for 1 minute every 10 minutes to echo coordinates.
```
# 8. Sleep Modes
```
Light Sleep:
Trigger: Resting ≥70% for 5–10 minutes.
Behavior: Log at 0.05 Hz (standard dog), BLE/Wi-Fi off, accelerometer at 10 Hz, ESP32-S3 light sleep (~0.5 mA).
Exit: Wake on accelerometer interrupt (movement).
```
```
Deep Sleep:
Trigger: Resting ≥70% for >10 minutes.
Behavior: Log at 0.01 Hz, BLE/Wi-Fi off, accelerometer at 1 Hz (approximating ULP), ESP32-S3 deep sleep (~10 µA).
Exit: Wake on accelerometer interrupt.
Note: Deep sleep currently uses 1 Hz accelerometer; true ULP with interrupts can reduce power further.
```
# 9. Dynamic Dog Type Detection
```
Logic: Profile activity for 3 days (259200 samples at 10-second intervals):
- Active Dog: >50% running.
- Standard Dog: ~30% walking, 10% running, 60% resting.
- Adult Dog: >80% resting.
Implementation: Calculate activity distribution, classify after 3 days, adjust frequencies.
Default: Use standard dog frequencies until classification.
```
# 10. Battery Life Feasibility

Budget: 350 mAh / 2880 hours (120 days) = ~0.12 mA average.
```
Consumption:
Active (40%): ~20 mA (ESP32-S3) + 0.1 mA (accelerometer, 100 Hz) = ~8 mA average.
Light Sleep (20%): ~0.5 mA (ESP32-S3) + 0.05 mA (accelerometer, 10 Hz) = ~0.11 mA average.
Deep Sleep (40%): ~0.01 mA (ESP32-S3) + 0.01 mA (accelerometer, 1 Hz) = ~0.008 mA average.
BLE (5%): ~5 mA × 0.05 = 0.25 mA average (1 minute every 10 minutes outdoors).
Wi-Fi (0.01%): ~100 mA × 0.0001 = 0.01 mA average (10 seconds every 12 hours).
Total: ~0.1–0.15 mA, within 350 mAh for ~120 days (standard dog).
Note: Active dogs (~90 days) use higher frequencies; adult dogs (~150 days) use lower.
```
