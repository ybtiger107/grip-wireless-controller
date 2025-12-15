# ppg_imu
**PPG + IMU raw data acquisition firmware**

This firmware is an **experimental module** used to directly acquire
raw IMU and PPG sensor data for validation, debugging, and signal analysis.
It is not intended for wireless transmission or real-time interaction,
but rather for sensor-level inspection and tuning.

---

## Purpose

`ppg_imu.ino` is designed to:

- Verify correct wiring and I2C communication
- Observe raw accelerometer and gyroscope data
- Inspect raw PPG (IR / Red) signals
- Tune sensor gain, range, and filter parameters
- Visualize signals using the Arduino Serial Plotter

This module was used during early-stage development of GRIP
to validate sensor behavior before integrating wireless communication
and advanced processing.

---

## Hardware Requirements

- ESP8266 (WeMos D1 mini)
- MPU6050 IMU (I2C)
- MAX30102 or MAX30105 PPG sensor
- USB connection to a host PC

---

## Software Dependencies

This firmware depends on the following Arduino libraries:

- `Wire`
- `Adafruit_MPU6050`
- `Adafruit_Sensor`
- `MAX30105`

Ensure all required libraries are installed before compiling.

---

## Sensor Configuration

### MPU6050
- Accelerometer range: ±4g
- Gyroscope range: ±500 deg/s
- Digital low-pass filter: 21 Hz

### MAX30102 / MAX30105
- IR LED enabled (primary PPG channel)
- Red LED enabled (optional, for SpO2-related experiments)
- Green LED disabled

---

## Output Format

Sensor data is printed via Serial in a format compatible with
the **Arduino Serial Plotter**:


