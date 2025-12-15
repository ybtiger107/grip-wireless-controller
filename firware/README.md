This folder contains three Arduino (.ino) files.

- **imu_ir_send.ino**  
  The main working firmware for the GRIP system.  
  It handles IMU data acquisition and IR-based click detection,
  and transmits the data wirelessly via UDP.

- **ppg_imu.ino**  
  An experimental firmware used for testing raw PPG and IMU sensor data,
  mainly for signal validation and debugging purposes.

- **wifi_udp_send.ino**  
  A lightweight firmware focused on optimizing Wi-Fi and UDP transmission
  performance for personal experiments.
