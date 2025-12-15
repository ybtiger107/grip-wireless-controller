# GRIP  
**Gesture Recognition via Integrated PPG**

GRIP is a wireless gesture recognition and input system that integrates
photoplethysmography (PPG) and an inertial measurement unit (IMU)
to capture fine-grained hand and grip intentions.
It enables intuitive human–computer interaction for
VR/AR, precision robotics, and rehabilitation applications.

---

## Motivation

Conventional gesture controllers rely primarily on IMU-based motion tracking.
While effective for orientation and gross movement, IMU-only approaches can suffer from
drift, noise, and limited ability to directly capture **user intention**
(e.g., grip/click actions).

GRIP addresses this limitation by integrating PPG sensing as an additional modality.
Physiological signals related to grip (e.g., blood volume changes and muscle activity effects)
can complement IMU motion information, enabling more intention-aware input.

---

## System Overview

GRIP consists of a wearable sensing module and a host-side software pipeline:

- **Wearable module (ESP8266 + sensors)**  
  Collects IMU orientation/motion and PPG(IR) signals and streams data via UDP
- **Host PC (Python)**  
  Receives data, visualizes motion, and maps gestures to mouse/input events

---

## Repository Structure

- **[`firmware/`](./firmware)**  
  ESP8266/Arduino firmware modules (UDP streaming, IMU+PPG integration, diagnostics)
- **[`host/`](./host)**  
  Python receiver + visualization + mouse control scripts  
- **[`hardware/`](./hardware)**  
  Hardware notes, wiring, and future documentation (placeholder)
- **[`docs/`](./docs)**  
  Slides, images, and documentation assets (placeholder)

---

## Quick Start

### 1) Firmware (ESP8266)
Use the main integrated firmware:

- **[`firmware/imu_ir_send/imu_ir_send.ino`](./firmware/imu_ir_send/imu_ir_send.ino)**

Make sure you set:
- Wi-Fi SSID / password  
- Host PC IP address and UDP port

### 2) Host (PC)
Run the final integrated host application:

- **[`host/GRIP.py`](./host/GRIP.py)**

This program:
- visualizes the Z-axis direction in VPython,
- calibrates monitor mapping (4 corners),
- calibrates fist (IR) thresholds,
- controls mouse movement and click.

---

## Network Debugging (Recommended)

If UDP reception is unstable, validate the network first:

1. Flash **[`firmware/wifi_udp_send/wifi_udp_send.ino`](./firmware/wifi_udp_send/wifi_udp_send.ino)**
2. Run **[`host/test_wifi.py`](./host/test_wifi.py)**

Once `"HELLO_FROM_D1_MINI"` is received reliably, switch back to `imu_ir_send.ino`.

---

## Key Modules

### Firmware
- **`imu_ir_send`**: main IMU + IR(PPG) integrated UDP sender (reference firmware)
- **`ppg_imu`**: sensor validation (raw IMU + PPG via Serial Plotter)
- **`wifi_udp_send`**: minimal Wi-Fi/UDP diagnostic sender

### Host
- **`GRIP.py`**: final integrated pipeline (visualization + calibration + mouse control)
- **`GRIP_test.py`**: alternative experimental pipeline (kept for reference)
- **`fist_space.py`**: standalone fist(IR) calibration + gesture prototype
- **`plot_integrated_xyz.py`**: deprecated experiment (double-integration drift)

---

## Applications

- Virtual / Augmented Reality interaction
- Precision robot control
- Assistive and rehabilitation technology
- Intuitive wearable input devices

---

## Key Contributions

- Integrated PPG(IR) sensing to capture grip/click intention beyond IMU-only tracking
- Low-latency UDP-based wireless sensing pipeline (ESP8266 → PC)
- Real-time visualization and input control prototype (VPython + mouse mapping)
- Practical calibration flow for display mapping and fist thresholds

---

## Limitations and Future Work

- Current implementation is a proof-of-concept prototype
- Gesture logic is primarily rule-based (thresholding)
- Future work includes:
  - improved filtering and feature extraction,
  - ML-based gesture classification,
  - hardware miniaturization,
  - formal latency/accuracy evaluation.

---

## License

This project is released under the MIT License.

---

## Contributors

- **Paul Sim**  
  System design, ESP8266 firmware development, wireless UDP communication,  
  Python-based visualization and input control

- **Yeji Seo**  
  PPG sensor integration, physiological signal experimentation,  
  gesture and grip interaction concept development  
  (seo040824@g.skku.edu)
