# Host (PC-side) Scripts

This folder contains Python scripts for receiving GRIP UDP packets,
visualizing orientation, and mapping gestures to mouse input on the host PC.

## Quick Start (Recommended)

1) **Firmware side**
- Upload and run: [`firmware/imu_ir_send/imu_ir_send.ino`](../firmware/imu_ir_send/imu_ir_send.ino)

2) **Host side**
- Run the main application: **[`GRIP.py`](./GRIP.py)**

If UDP reception is unstable or you suspect network issues,
first validate the connection using:
- [`firmware/wifi_udp_send/wifi_udp_send.ino`](../firmware/wifi_udp_send/wifi_udp_send.ino)
- **[`test_wifi.py`](./test_wifi.py)**

---

## Files in this folder

### ✅ GRIP.py (Main / Final)
- **Final integrated host program** for GRIP
- Receives UDP packets (11 fields) from the firmware
- VPython visualization of Z-axis direction
- Monitor + fist (IR) calibration flow
- Mouse cursor mapping to the target monitor (Windows multi-monitor supported)

**Calibration flow inside GRIP.py**
1. Press **`C`** to start calibration
2. **Monitor calibration**: aim at 4 corners and press **SPACE** to save each point
3. **Fist calibration**: repeatedly clench/release for a fixed duration
4. RUN mode: mouse moves by orientation + click by IR

> Notes: `GRIP.py` uses Windows monitor enumeration (`ctypes`) to target a specific display.

---

### 🧪 GRIP_test.py (Not used / Alternative)
- Experimental variant with additional smoothing/compensation ideas
- Kept for reference, but **not adopted as the final pipeline**
- Uses single-monitor screen mapping via `tkinter` (simpler, less flexible than GRIP.py)

---

### 🧪 fist_space.py (Calibration prototype)
- Standalone IR calibration + fist gesture detector
- Collects IR values for a few seconds, estimates thresholds,
  and triggers **space key** on detected fist gesture
- Useful to debug IR click detection logic in isolation

---

### 🧪 plot_integrated_xyz.py (Deprecated experiment)
- Early attempt to estimate 3D position by double-integrating acceleration
- Abandoned due to drift/noise and instability
- Kept as a record of experimental exploration

---

### ✅ test_wifi.py (Network debug tool)
- Minimal UDP receiver that prints incoming packets
- Use this together with:
  - [`firmware/wifi_udp_send`](../firmware/wifi_udp_send)
- Helps quickly identify:
  - wrong host IP / wrong port
  - Wi-Fi connection issues
  - firewall / network isolation problems

---

## Dependencies

Recommended Python packages:

- `numpy`
- `keyboard`
- `pynput`
- `vpython`
- (GRIP_test.py only) `tkinter` (usually included with Python on Windows)

Install (example):
- `pip install numpy keyboard pynput vpython`

> `keyboard` may require running the terminal as Administrator on Windows.

---

## UDP Packet Format (Expected)

GRIP firmware sends a single comma-separated line with **11 fields**:

`t_ms, ax, ay, az, dx, dy, dz, ir, yaw, pitch, roll`

- `dx, dy, dz` : Z-axis direction vector (used for cursor mapping + VPython)
- `ir`         : IR value (used for fist/click detection)

---

## Troubleshooting Checklist

1) No packets received?
- Run [`test_wifi.py`](./test_wifi.py) first
- Use [`wifi_udp_send`](../firmware/wifi_udp_send) to isolate network issues

2) Cursor moves on the wrong monitor?
- In `GRIP.py`, adjust `TARGET_MONITOR_INDEX`

3) Click detection unstable?
- Use `fist_space.py` to validate IR thresholds independently

