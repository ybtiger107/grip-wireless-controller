# GRIP  
**Gesture Recognition via Integrated PPG**

GRIP is a wireless gesture recognition system that integrates
photoplethysmography (PPG) and inertial measurement unit (IMU) sensors
to capture fine-grained hand and grip intentions.
The system enables intuitive human–computer interaction for
VR/AR, precision robotics, and rehabilitation applications.

---

## Motivation

Conventional wireless input devices and gesture controllers rely
primarily on IMU-based motion tracking.
While effective for orientation and gross movement, IMU-only systems
suffer from drift, noise, and limited ability to directly capture
user intention such as grip or click actions.

GRIP addresses this limitation by integrating PPG sensing,
allowing physiological signals related to muscle contraction and blood
volume changes to be used as an additional interaction modality.
This enables more intuitive, robust, and intention-aware input.

---

## System Overview

GRIP consists of a wearable wireless sensing module and a host-side
software pipeline:

- **PPG sensor**  
  Detects grip and click-related physiological signals
- **IMU (MPU)**  
  Captures 3D orientation and motion
- **ESP8266 (WeMos D1 mini)**  
  Performs sensor fusion and transmits data wirelessly via UDP
- **Host PC (Python)**  
  Receives data, visualizes motion, and maps gestures to input events

---

## Hardware Components

- ESP8266 (WeMos D1 mini)
- PPG sensor (IR-based)
- IMU (MPU series)
- Battery-powered wearable form factor

---

## Software Architecture

### Embedded Firmware (ESP8266)
- Sensor acquisition (PPG + IMU)
- Basic preprocessing
- UDP packet transmission over Wi-Fi

### Host-side Software (Python)
- UDP data reception
- Real-time visualization (2D / 3D)
- Gesture interpretation
- Mouse / input control experiments

Multiple experimental scripts were developed to test:
- IMU-only tracking
- PPG-assisted gesture detection
- 3D orientation visualization
- Wireless latency and stability

---

## Experimental Files (Current State)

This repository contains multiple experimental and iterative files,
reflecting the development and testing process:

- **ESP8266 firmware**
  - IMU-only transmission
  - PPG + IMU integrated transmission
  - UDP-based wireless streaming
- **Python scripts**
  - UDP receiver variants
  - 2D/3D plotting and visualization
  - Custom gesture and input mapping tests

File naming reflects versioned experiments rather than final structure.
Folder reorganization and cleanup are planned as future work.

---

## Applications

- Virtual / Augmented Reality interaction
- Precision robot control
- Assistive and rehabilitation technology
- Intuitive wearable input devices

---

## Key Contributions

- Integration of PPG sensing for gesture and grip intention recognition
- Wireless UDP-based low-latency input pipeline
- Exploration of physiological signals as an alternative to button-based input
- Experimental validation through multiple visualization and control prototypes

---

## Limitations and Future Work

- Current implementation focuses on proof-of-concept experiments
- Signal processing and classification are rule-based
- Future work includes:
  - Improved signal filtering and feature extraction
  - Machine learning–based gesture classification
  - Hardware miniaturization
  - Formal latency and accuracy evaluation

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
