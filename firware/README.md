## Firmware Overview

This folder contains three Arduino (.ino) firmware modules
used in the GRIP system.

- **[imu_ir_send](./imu_ir_send)**  
  Main working firmware for GRIP.  
  Handles IMU data acquisition and IR-based click detection,
  and transmits the data wirelessly via UDP.  
  → Entry file: [`imu_ir_send.ino`](./imu_ir_send/imu_ir_send.ino)

- **[ppg_imu](./ppg_imu)**  
  Experimental firmware for testing raw PPG and IMU sensor data,
  mainly used for signal validation and debugging.  
  → Entry file: [`ppg_imu.ino`](./ppg_imu/ppg_imu.ino)

- **[wifi_udp_send](./wifi_udp_send)**  
  Lightweight firmware focused on optimizing Wi-Fi and UDP
  transmission performance for personal experiments.  
  → Entry file: [`wifi_udp_send.ino`](./wifi_udp_send/wifi_udp_send.ino)
