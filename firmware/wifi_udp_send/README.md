# wifi_udp_send
**Minimal Wi-Fi + UDP transmission test firmware**

This firmware is a **network diagnostic and validation module**
for the GRIP system.
It is used to verify Wi-Fi connectivity, UDP transmission,
and host-side reception before running sensor-integrated firmware.

---

## Purpose

`wifi_udp_send.ino` is designed to:

- Confirm successful Wi-Fi connection on ESP8266
- Verify correct UDP packet transmission
- Test host-side UDP receiver scripts
- Debug network-related issues independently of sensor logic

This module sends a simple text message at a fixed interval,
making it ideal for isolating communication problems.

---

## Hardware Requirements

- ESP8266 (WeMos D1 mini)
- Wi-Fi access point (hotspot or router)
- Host PC on the same network

---

## Network Configuration

Update the following parameters in the source code:

```cpp
const char* ssid       = "YOUR_WIFI_SSID";
const char* password   = "YOUR_WIFI_PASSWORD";
const char* udpAddress = "PC_IP_ADDRESS";
const int   udpPort    = 9000;

