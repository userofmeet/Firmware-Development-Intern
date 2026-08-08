# Firmware Development Intern at Marshee Pet Tech

## Title:  
**Microcontroller-Based Display and Sensor Interface Using ESP32-S3 1.69" Display Development Board**

## Intern:  
**Meet Jain**  
B.Tech Electronics & Communication Engineering  
Dharmsinh Desai University, Nadiad

## Internship Duration:  
**5th May** – **28th June**

---

## Abstract

This internship focused on developing an embedded system prototype for a startup using the ESP32-S3 1.69" Display Development Board by Waveshare. The board integrates a 240MHz dual-core LX7 processor, a 240×280 pixel LCD, onboard Wi-Fi/Bluetooth, and a 6-axis IMU. The system was tested for real-time sensor display, motion tracking, and user interface (UI) elements using C/C++ and the Arduino IDE. This compact yet powerful platform enabled fast prototyping for wearables, IoT dashboards, and interactive interfaces.

---

## Keywords

ESP32-S3, LCD Display, IoT, Bluetooth, Wi-Fi, BLE Beacon, Microcontroller, IMU, Motion Tracking, Embedded Systems, Arduino, GPS Integration

---

## Introduction

The ESP32-S3 is an advanced microcontroller offering high-performance computing, wireless communication, and multimedia capabilities. The development board used in this project features:
- **1.69" IPS LCD Display** (240×280 resolution, 262K color)
- **Dual-Core Xtensa LX7 processor** up to 240 MHz
- **Wi-Fi and Bluetooth 5 (LE)**
- **QMI8658 IMU sensor** (3-axis accelerometer + 3-axis gyroscope)
- **RTC (PCF85063)**
- **8MB PSRAM, 16MB Flash**

This project explores how such a compact platform can be utilized for sensor data acquisition, BLE applications, and visual feedback using GPS and display modules.

---

## Internship Tasks

As part of the internship, the following key tasks were undertaken:

1. **Location Services Using GPS of Paired Mobile Phone**  
   - Developed integration logic to fetch GPS coordinates from a paired smartphone via Bluetooth.
   - Displayed location data on the onboard screen.
   - Worked on parsing and formatting NMEA/JSON-based location data from the mobile device.

2. **BLE Beacon Integration for Apple and Google Devices**  
   - Implemented iBeacon and Eddystone beacon protocols using ESP32-S3 BLE capabilities.
   - Successfully tested beacon broadcasting and visibility on iOS and Android devices.
   - Optimized BLE intervals and payloads for minimal power consumption.

3. **Optimization of Other Modules on the Device**  
   - Optimized the LCD SPI refresh rate to reduce latency and flicker.
   - Tuned sensor readout frequencies to maintain a balance between responsiveness and efficiency.
   - Reduced memory usage by optimizing data buffers and screen refresh logic.

These tasks helped demonstrate the potential of the ESP32-S3 as a multifunctional embedded controller for IoT, wearable, and location-based solutions.

---

## Tools and Technologies Used

| Tool/Technology | Purpose |
|-----------------|---------|
| ESP32-S3        | Core microcontroller |
| Arduino IDE / PlatformIO | Development environment |
| C/C++           | Programming language |
| SPI Protocol    | LCD Communication |
| QMI8658         | IMU sensor for motion tracking |
| BLE (iBeacon/Eddystone) | Beacon broadcast protocol |
| GPS-over-Bluetooth | Mobile phone location sharing |
| Waveshare Wiki  | Hardware reference |

---

## Experimental Setup

### Hardware Used:
- **ESP32-S3 1.69" LCD Display Dev Board**
- **3.7V Lithium Battery (via MX1.25 header)**
- **USB-C Cable for programming**
- **Android/iOS smartphone for BLE & GPS**
- **Computer with Arduino IDE**

### Board Features:
- Onboard **QMI8658 IMU**
- **RTC (PCF85063)**
- **PWM button** for UI/interrupts
- **Type-C connector** for power & programming
- **12-pin GPIO header** for expansions
- **ST7789V2 Display Driver** via SPI

---
