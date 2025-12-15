import socket
import time
import numpy as np
import keyboard
from pynput.mouse import Controller, Button
from vpython import *
import ctypes
from ctypes import wintypes

# =========================================================
# [0] Windows: 모니터(디스플레이) 정보 가져오기
# =========================================================
user32 = ctypes.WinDLL("user32", use_last_error=True)

MONITORINFOF_PRIMARY = 0x00000001

class RECT(ctypes.Structure):
    _fields_ = [("left", ctypes.c_long),
                ("top", ctypes.c_long),
                ("right", ctypes.c_long),
                ("bottom", ctypes.c_long)]

class MONITORINFO(ctypes.Structure):
    _fields_ = [("cbSize", wintypes.DWORD),
                ("rcMonitor", RECT),
                ("rcWork", RECT),
                ("dwFlags", wintypes.DWORD)]

MonitorEnumProc = ctypes.WINFUNCTYPE(
    wintypes.BOOL,
    wintypes.HMONITOR,
    wintypes.HDC,
    ctypes.POINTER(RECT),
    wintypes.LPARAM
)

def list_monitors():
    monitors = []

    def _callback(hMonitor, hdcMonitor, lprcMonitor, dwData):
        mi = MONITORINFO()
        mi.cbSize = ctypes.sizeof(MONITORINFO)
        user32.GetMonitorInfoW(hMonitor, ctypes.byref(mi))
        r = mi.rcMonitor
        x0, y0 = r.left, r.top
        w, h = (r.right - r.left), (r.bottom - r.top)
        is_primary = bool(mi.dwFlags & MONITORINFOF_PRIMARY)
        monitors.append({"x0": x0, "y0": y0, "w": w, "h": h, "primary": is_primary})
        return True

    user32.EnumDisplayMonitors(0, 0, MonitorEnumProc(_callback), 0)
    return monitors

# ---- TV(타겟) 모니터 선택 옵션 ----
TARGET_MONITOR_INDEX = 1  # TV가 2번째로 감지되면 1

monitors = list_monitors()
if not monitors:
    raise RuntimeError("모니터 정보를 가져오지 못했습니다.")

print("감지된 모니터들:")
for i, m in enumerate(monitors):
    tag = "PRIMARY" if m["primary"] else ""
    print(f"  [{i}] x0={m['x0']}, y0={m['y0']}, w={m['w']}, h={m['h']} {tag}")

if TARGET_MONITOR_INDEX is not None and 0 <= TARGET_MONITOR_INDEX < len(monitors):
    target = monitors[TARGET_MONITOR_INDEX]
else:
    # 기본: 가장 큰 면적을 TV로 가정
    target = max(monitors, key=lambda d: d["w"] * d["h"])

TV_X0, TV_Y0, TV_W, TV_H = target["x0"], target["y0"], target["w"], target["h"]
print(f"\n✅ 타겟(=TV) 모니터: x0={TV_X0}, y0={TV_Y0}, w={TV_W}, h={TV_H}\n")

# =========================================================
# [1] UDP 통신 설정
# =========================================================
UDP_IP = "0.0.0.0"
UDP_PORT = 9000
UDP_TIMEOUT = 0.01

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(UDP_TIMEOUT)

print(f"UDP 수신 대기중... (포트 {UDP_PORT})")
print("Z축 방향벡터(dx,dy,dz)로 마우스 + VPython 시각화, IR로 클릭 제어를 합니다.")

mouse = Controller()

# =========================================================
# [2] VPython 3D 캔버스 & 좌표축
# =========================================================
scene = canvas(
    title='Sensor Z-axis Direction (EMA smoothing + Mouse Control)',
    width=800, height=600,
    center=vector(0, 0, 0),
    background=color.cyan
)

arrow(pos=vector(0,0,0), axis=vector(1,0,0), color=color.red,   length=1.0, shaftwidth=0.02)
arrow(pos=vector(0,0,0), axis=vector(0,1,0), color=color.green, length=1.0, shaftwidth=0.02)
arrow(pos=vector(0,0,0), axis=vector(0,0,1), color=color.blue,  length=1.0, shaftwidth=0.02)

label(pos=vector(1.1,0,0), text="World X", color=color.red)
label(pos=vector(0,1.1,0), text="World Y", color=color.green)
label(pos=vector(0,0,1.1), text="World Z", color=color.blue)

scene.autoscale = False

z_axis_arrow = arrow(
    pos=vector(0,0,0),
    axis=vector(0,0,1),
    length=1.0,
    shaftwidth=0.06,
    color=color.yellow
)

info_label = label(pos=vector(0, 1.6, 0), text="Waiting for data...", color=color.white)

# =========================================================
# [3] 모드 및 캘리브레이션 관련 전역 변수
# =========================================================
MODE = "WAIT_CALIB"   # "WAIT_CALIB" -> "MONITOR_CALIB" -> "FIST_CALIB" -> "RUN"

MONITOR_CALIB_POINTS = ["TOP-LEFT", "TOP-RIGHT", "BOTTOM-RIGHT", "BOTTOM-LEFT"]
calib_step_index = 0
monitor_data = {}

min_yaw_a, max_yaw_a = 0.0, 0.0
min_pitch_a, max_pitch_a = 0.0, 0.0

ARROW_LENGTH = 1.0
filtered_dir = vector(0, 0, 1)
ALPHA_BASE = 0.2
last_t_ms = None

ROLL_FILTER_THRESHOLD = 45.0

MIN_IR_CONTACT = 10000
ALPHA_BASELINE = 0.01
FIST_THRESH_HIGH = 7000.0
FIST_THRESH_LOW = 4500.0
FIST_CALIB_DURATION = 10.0

baseline_ir = 100000.0
fist_state = "IDLE"
is_mouse_down = False

fist_calib_start = None
fist_ir_list = []

# ✅ MONITOR_CALIB 콘솔 스팸 방지(한 줄 갱신)
MONITOR_PRINT_PERIOD = 0.10  # 초
_last_monitor_print_t = 0.0

# =========================================================
# [4] UDP 패킷 파싱
#   포맷: t_ms, ax, ay, az, dx, dy, dz, ir, yaw, pitch, roll
# =========================================================
def recv_packet():
    try:
        data, addr = sock.recvfrom(1024)
    except socket.timeout:
        return None

    msg = data.decode("utf-8", errors="ignore").strip()
    parts = msg.split(",")

    if len(parts) != 11:
        return None

    try:
        t_ms   = float(parts[0])
        ax     = float(parts[1])
        ay     = float(parts[2])
        az     = float(parts[3])
        dx     = float(parts[4])
        dy     = float(parts[5])
        dz     = float(parts[6])
        ir     = int(parts[7])
        yaw    = float(parts[8])
        pitch  = float(parts[9])
        roll   = float(parts[10])
    except ValueError:
        return None

    return t_ms, ax, ay, az, dx, dy, dz, ir, yaw, pitch, roll

# =========================================================
# [5] 방향벡터 → (yaw_like, pitch_like)
# =========================================================
def dir_to_angles(v: vector):
    x, y, z = v.x, v.y, v.z
    yaw_a = np.arctan2(x, z)
    pitch_a = np.arctan2(y, np.hypot(x, z))
    return yaw_a, pitch_a

# =========================================================
# [6] MONITOR 캘리브레이션
# =========================================================
def calibrate_monitor(ir, dir_vec):
    global MODE, calib_step_index
    global min_yaw_a, max_yaw_a, min_pitch_a, max_pitch_a, monitor_data
    global _last_monitor_print_t

    yaw_a, pitch_a = dir_to_angles(dir_vec)
    current_point = MONITOR_CALIB_POINTS[calib_step_index]

    # ✅ 콘솔 출력: 0.1초마다 한 번만 "한 줄"로 갱신
    now = time.time()
    if now - _last_monitor_print_t >= MONITOR_PRINT_PERIOD:
        _last_monitor_print_t = now
        ir_str = f"{ir}" if ir is not None else "None"
        print(
            f"[MONITOR CALIB] Target={current_point:12s} | "
            f"yaw={np.degrees(yaw_a):7.2f}° | pitch={np.degrees(pitch_a):7.2f}° | IR={ir_str:>6s} "
            f"| (SPACE to save)",
            end="\r"
        )

    if keyboard.is_pressed('space'):
        monitor_data[current_point] = (yaw_a, pitch_a)
        print(f"\n✅ [{current_point}] 기록 완료. (yaw={np.degrees(yaw_a):.2f}°, pitch={np.degrees(pitch_a):.2f}°, IR={ir})")
        calib_step_index += 1
        time.sleep(0.5)

        if calib_step_index >= len(MONITOR_CALIB_POINTS):
            x_min_yaw = (monitor_data['TOP-LEFT'][0] + monitor_data['BOTTOM-LEFT'][0]) / 2
            x_max_yaw = (monitor_data['TOP-RIGHT'][0] + monitor_data['BOTTOM-RIGHT'][0]) / 2

            y_min_pitch = (monitor_data['TOP-LEFT'][1] + monitor_data['TOP-RIGHT'][1]) / 2
            y_max_pitch = (monitor_data['BOTTOM-LEFT'][1] + monitor_data['BOTTOM-RIGHT'][1]) / 2

            min_yaw_a = min(x_min_yaw, x_max_yaw)
            max_yaw_a = max(x_min_yaw, x_max_yaw)
            min_pitch_a = min(y_min_pitch, y_max_pitch)
            max_pitch_a = max(y_min_pitch, y_max_pitch)

            print("\n=== MONITOR CALIBRATION 완료 ===")
            print(f"Yaw_like 범위: {np.degrees(min_yaw_a):.1f}° ~ {np.degrees(max_yaw_a):.1f}°")
            print(f"Pitch_like 범위: {np.degrees(min_pitch_a):.1f}° ~ {np.degrees(max_pitch_a):.1f}°")

            MODE = "FIST_CALIB"
            calib_step_index = 0
            print("\n>>> FIST 캘리브레이션으로 전환: 주먹 쥐었다 펴세요!")
            time.sleep(1.0)

# =========================================================
# [7] FIST 캘리브레이션
# =========================================================
def calibrate_fist(ir):
    global MODE, baseline_ir, FIST_THRESH_HIGH, FIST_THRESH_LOW
    global fist_calib_start, fist_ir_list

    if fist_calib_start is None:
        fist_calib_start = time.time()
        fist_ir_list = []
        print(f"\n-> FIST CALIB: {FIST_CALIB_DURATION}초 동안 주먹을 쥐었다 폈다 반복하세요.")

    elapsed = time.time() - fist_calib_start
    if elapsed < FIST_CALIB_DURATION:
        if ir is not None:
            if ir > MIN_IR_CONTACT:
                fist_ir_list.append(ir)
            print(f"현재 IR 값: {ir}", end='\r')
        return

    if len(fist_ir_list) < 100:
        print("\n캘리브레이션 데이터가 너무 적습니다. FIST 캘리브레이션 재시도하세요.")
        fist_calib_start = None
        fist_ir_list = []
        return

    ir_sorted = sorted(fist_ir_list)
    n = len(ir_sorted)
    low_slice = ir_sorted[:max(1, n // 5)]
    baseline = sum(low_slice) / len(low_slice)
    high_slice = ir_sorted[-max(1, n // 5):]
    peak = sum(high_slice) / len(high_slice)
    delta = max(peak - baseline, 1000.0)

    baseline_ir = baseline
    FIST_THRESH_HIGH = baseline + delta * 0.7
    FIST_THRESH_LOW  = baseline + delta * 0.5

    print("\n=== FIST CALIBRATION 완료 ===")
    print(f"baseline_ir = {baseline_ir:.1f}")
    print(f"FIST_THRESH_HIGH = {FIST_THRESH_HIGH:.1f}")
    print(f"FIST_THRESH_LOW  = {FIST_THRESH_LOW:.1f}")

    MODE = "RUN"
    fist_calib_start = None
    fist_ir_list = []

    print("===========================================")
    print("🔥 마우스 제어 시작! (Z축 방향벡터 + 모니터 매핑 + FIST 클릭)")
    print("===========================================")
    time.sleep(1.0)

# =========================================================
# [8] RUN 모드: 마우스 움직임 + 클릭
# =========================================================
def run_control(ir, dir_vec):
    global baseline_ir, fist_state, is_mouse_down
    global min_yaw_a, max_yaw_a, min_pitch_a, max_pitch_a
    global TV_X0, TV_Y0, TV_W, TV_H

    # A. 마우스 움직임 (Z축 방향벡터 -> 각도 -> TV 화면 좌표)
    if min_yaw_a != max_yaw_a and min_pitch_a != max_pitch_a:
        yaw_a, pitch_a = dir_to_angles(dir_vec)

        normalized_x = (yaw_a - min_yaw_a) / (max_yaw_a - min_yaw_a)
        normalized_y = (pitch_a - min_pitch_a) / (max_pitch_a - min_pitch_a)

        # 필요하면 반전:
        # normalized_x = 1.0 - normalized_x
        # normalized_y = 1.0 - normalized_y

        normalized_x = np.clip(normalized_x, 0.0, 1.0)
        normalized_y = np.clip(normalized_y, 0.0, 1.0)

        target_x = TV_X0 + normalized_x * TV_W
        target_y = TV_Y0 + normalized_y * TV_H

        mouse.position = (target_x, target_y)

    # B. 마우스 클릭 (IR 기반)
    if ir is None:
        if is_mouse_down:
            mouse.release(Button.left)
            is_mouse_down = False
            fist_state = "IDLE"
        return

    if ir < MIN_IR_CONTACT:
        if is_mouse_down:
            mouse.release(Button.left)
            is_mouse_down = False
        fist_state = "IDLE"
        return

    if fist_state == "IDLE":
        baseline_ir = (1 - ALPHA_BASELINE) * baseline_ir + ALPHA_BASELINE * ir
        if ir > FIST_THRESH_HIGH:
            fist_state = "FIST"
            if not is_mouse_down:
                mouse.press(Button.left)
                is_mouse_down = True
    elif fist_state == "FIST":
        if ir < FIST_THRESH_LOW:
            fist_state = "IDLE"
            if is_mouse_down:
                mouse.release(Button.left)
                is_mouse_down = False

# =========================================================
# [9] 시작 안내
# =========================================================
print("===========================================")
print(f"현재 모드: {MODE}")
print("-> 'C' 키를 눌러 캘리브레이션을 시작하세요.")
print("===========================================")
print("### 캘리브레이션 순서: MONITOR(4 꼭짓점) -> FIST(클릭) -> RUN ###")

# =========================================================
# [10] 메인 루프
# =========================================================
while True:
    rate(100)

    pkt = recv_packet()
    if pkt is None:
        if MODE == "RUN" and is_mouse_down:
            mouse.release(Button.left)
            is_mouse_down = False
            fist_state = "IDLE"
        continue

    t_ms, ax, ay, az, dx, dy, dz, ir, yaw, pitch, roll = pkt

    v = vector(dx, dy, dz)
    if mag(v) < 1e-9:
        continue

    new_dir = norm(v)

    if last_t_ms is None:
        dt_ms = 0.0
    else:
        dt_ms = t_ms - last_t_ms
    last_t_ms = t_ms

    target_dt_ms = 20.0
    scale = dt_ms / target_dt_ms if target_dt_ms > 0 else 1.0
    scale = max(0.5, min(scale, 1.5))
    alpha = ALPHA_BASE * scale
    alpha = max(0.0, min(alpha, 1.0))

    filtered_dir = (1.0 - alpha) * filtered_dir + alpha * new_dir
    if mag(filtered_dir) < 1e-9:
        filtered_dir = new_dir
    else:
        filtered_dir = norm(filtered_dir)

    z_axis_arrow.axis = filtered_dir * ARROW_LENGTH

    # ✅ VPython 라벨에도 yaw/pitch + IR + 현재 캘리브 포인트 표시
    yaw_like, pitch_like = dir_to_angles(filtered_dir)
    t_s = t_ms / 1000.0

    calib_target = "-"
    if MODE == "MONITOR_CALIB":
        calib_target = MONITOR_CALIB_POINTS[calib_step_index]

    info_label.text = (
        f"MODE: {MODE}\n"
        f"CALIB target: {calib_target}\n"
        f"Sensor time: {t_s:.3f} s  (Δt = {dt_ms:.1f} ms)\n"
        f"yaw_like: {np.degrees(yaw_like):.2f}°, pitch_like: {np.degrees(pitch_like):.2f}°\n"
        f"EMA alpha: {alpha:.3f}\n"
        f"IR: {ir}"
    )

    if MODE == "WAIT_CALIB":
        if keyboard.is_pressed('c'):
            MODE = "MONITOR_CALIB"
            calib_step_index = 0
            print("\n>>> MONITOR 캘리브레이션 시작! (화면 4 꼭짓점)")
            time.sleep(1.0)

    elif MODE == "MONITOR_CALIB":
        calibrate_monitor(ir, filtered_dir)

    elif MODE == "FIST_CALIB":
        calibrate_fist(ir)

    elif MODE == "RUN":
        run_control(ir, filtered_dir)
