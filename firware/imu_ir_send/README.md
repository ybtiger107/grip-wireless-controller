# imu_ir_send
**IMU + IR based wireless sender firmware for GRIP**

This firmware is the **main working module** of the GRIP system.
It integrates IMU-based motion tracking with IR-based physiological sensing
and transmits the data wirelessly to a host PC via UDP.

---

## Overview

`imu_ir_send.ino` runs on an ESP8266 (WeMos D1 mini) and performs the following tasks:

- Initializes and calibrates the MPU6050 IMU using the DMP
- Computes orientation (Yaw, Pitch, Roll) and local Z-axis direction
- Calculates gravity-compensated linear acceleration (m/s²)
- Reads IR values from a MAX30102/30105 PPG sensor
- Streams all sensor data to a host PC via UDP in real time

This firmware serves as the **reference implementation** for GRIP
and is intended for demonstrations, experiments, and system integration.

---

## Hardware Requirements

- ESP8266 (WeMos D1 mini)
- MPU6050 IMU (I2C)
- MAX30102 or MAX30105 PPG sensor (optional but recommended)
- Stable Wi-Fi connection
- Battery-powered wearable setup (recommended)

---

## Software Dependencies

This firmware depends on the following Arduino libraries:

- `ESP8266WiFi`
- `WiFiUdp`
- `I2Cdev`
- `MPU6050_6Axis_MotionApps20`
- `MAX30105`

Make sure all required libraries are installed before compiling.

---

## Network Configuration

Update the following parameters in the source code to match your environment:

```cpp
const char* ssid       = "YOUR_WIFI_SSID";
const char* password   = "YOUR_WIFI_PASSWORD";
const char* udpAddress = "PC_IP_ADDRESS";
const int   udpPort    = 9000;
---

## Network Debugging and Validation

Before running this firmware, it is **strongly recommended**
to verify Wi-Fi and UDP communication using the
[`wifi_udp_send`](../wifi_udp_send) firmware.

This step helps isolate network-related issues
before introducing sensor processing complexity.

### Recommended Procedure

1. Upload the `wifi_udp_send.ino` firmware to the ESP8266
2. On the host PC, run the following script:

   - **[`test_wifi.py`](../../host/test_wifi.py)**

3. Confirm that the message `"HELLO_FROM_D1_MINI"` is received repeatedly
4. If packets are not received, check:
   - Wi-Fi SSID and password
   - Host PC IP address
   - UDP port number
   - Firewall or network isolation settings

Once basic UDP communication is confirmed,
switch to `imu_ir_send.ino` for full sensor-integrated testing.
