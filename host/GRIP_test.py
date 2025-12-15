import socket
import time
import numpy as np
import keyboard
import tkinter as tk
from pynput.mouse import Controller, Button
from vpython import *

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
# [2] 화면 정보
# =========================================================
def get_screen_size():
    try:
        root = tk.Tk()
        root.withdraw()
        width = root.winfo_screenwidth()
        height = root.winfo_screenheight()
        root.destroy()
        return width, height
    except Exception:
        return 1920, 1080

SCREEN_W, SCREEN_H = get_screen_size()
print(f"화면 해상도 감지: {SCREEN_W} x {SCREEN_H}")

# =========================================================
# [3] VPython 3D 캔버스 & 좌표축
# =========================================================
scene = canvas(
    title='Sensor Z-axis Direction (Smoothed + Mouse Control)',
    width=800, height=600,
    center=vector(0, 0, 0),
    background=color.cyan
)

# 월드 좌표축
arrow(pos=vector(0,0,0), axis=vector(1,0,0), color=color.red,   length=1.0, shaftwidth=0.02)
arrow(pos=vector(0,0,0), axis=vector(0,1,0), color=color.green, length=1.0, shaftwidth=0.02)
arrow(pos=vector(0,0,0), axis=vector(0,0,1), color=color.blue,  length=1.0, shaftwidth=0.02)

label(pos=vector(1.1,0,0), text="World X", color=color.red)
label(pos=vector(0,1.1,0), text="World Y", color=color.green)
label(pos=vector(0,0,1.1), text="World Z", color=color.blue)

scene.autoscale = False  # 줌 고정

# 센서 Z축 방향 화살표 (노란색)
z_axis_arrow = arrow(
    pos=vector(0,0,0),
    axis=vector(0,0,1),
    length=1.0,
    shaftwidth=0.06,
    color=color.yellow
)

# 정보 레이블
info_label = label(pos=vector(0, 1.5, 0), text="Waiting for data...", color=color.white)

# =========================================================
# [4] 모드 및 캘리브레이션 관련 전역 변수
# =========================================================
MODE = "WAIT_CALIB"   # "WAIT_CALIB" -> "MONITOR_CALIB" -> "FIST_CALIB" -> "RUN"

MONITOR_CALIB_POINTS = ["TOP-LEFT", "TOP-RIGHT", "BOTTOM-RIGHT", "BOTTOM-LEFT"]
calib_step_index = 0

monitor_data = {}  # 각 꼭짓점에서 측정한 (yaw_like, pitch_like)

# 방향벡터를 각도로 변환한 값의 범위
min_yaw_a, max_yaw_a = 0.0, 0.0
min_pitch_a, max_pitch_a = 0.0, 0.0

# =========================================================
# [5] 방향 벡터 스무딩 설정
# =========================================================
ARROW_LENGTH = 1.0
filtered_dir = vector(0, 0, 1)   # 초기 방향 (월드 Z)
last_t_ms = None

# 시간 상수 기반 EMA
TAU_MS = 35.0         # 타임 콘스턴트 (20~60ms 정도에서 조절)
SMOOTH_STEPS = 3      # 프레임 내 보간 스텝 수 (2~5 추천)
ANGLE_THRESH = np.radians(2.0)  # 2도 이하 변화는 무시 (미세 떨림 제거)

# =========================================================
# [6] IR / 클릭 관련
# =========================================================
ROLL_FILTER_THRESHOLD = 45.0     # (지금은 roll 사용 안 하지만 그대로 둠)

MIN_IR_CONTACT = 10000
ALPHA_BASELINE = 0.01
FIST_THRESH_HIGH = 7000.0
FIST_THRESH_LOW = 4500.0
FIST_CALIB_DURATION = 5.0

baseline_ir = 100000.0
fist_state = "IDLE"
is_mouse_down = False

# FIST 캘리브레이션용
fist_calib_start = None
fist_ir_list = []

# =========================================================
# [7] UDP 패킷 파싱
#   포맷: t_ms, ax, ay, az, dx, dy, dz, ir, yaw, pitch, roll
# =========================================================
def recv_packet():
    """
    UDP 패킷을 받아 전체 값을 반환.
    (없으면 None)
    """
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
# [8] 방향벡터 → (yaw_like, pitch_like) 변환 함수
# =========================================================
def dir_to_angles(v: vector):
    """
    방향벡터 v(dx,dy,dz)를
    - yaw_like: 수평 각도 (x-z 평면에서의 방향)
    - pitch_like: 수직 각도
    로 변환 (라디안 단위).
    """
    x, y, z = v.x, v.y, v.z
    yaw_a = np.arctan2(x, z)                   # -pi ~ pi
    pitch_a = np.arctan2(y, np.hypot(x, z))    # -pi/2 ~ pi/2
    return yaw_a, pitch_a

# =========================================================
# [9] MONITOR 캘리브레이션
# =========================================================
def calibrate_monitor(ir, dir_vec):
    global MODE, calib_step_index
    global min_yaw_a, max_yaw_a, min_pitch_a, max_pitch_a, monitor_data

    yaw_a, pitch_a = dir_to_angles(dir_vec)
    current_point = MONITOR_CALIB_POINTS[calib_step_index]

    print(f"-> MONITOR CALIB: {current_point} 지점 조준 후 SPACEBAR를 누르세요.")
    print(f"   (dir yaw_like, pitch_like) = ({np.degrees(yaw_a):.1f}°, {np.degrees(pitch_a):.1f}°)")

    if keyboard.is_pressed('space'):
        monitor_data[current_point] = (yaw_a, pitch_a)
        print(f"   [{current_point}] 기록 완료.")
        calib_step_index += 1
        time.sleep(0.5)

        if calib_step_index >= len(MONITOR_CALIB_POINTS):
            # 왼쪽/오른쪽, 위/아래 평균으로 범위 계산
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
# [10] FIST 캘리브레이션 (점진적)
# =========================================================
def calibrate_fist(ir):
    global MODE, baseline_ir, FIST_THRESH_HIGH, FIST_THRESH_LOW
    global fist_calib_start, fist_ir_list

    # 첫 호출 시 초기화
    if fist_calib_start is None:
        fist_calib_start = time.time()
        fist_ir_list = []
        print(f"\n-> FIST CALIB: {FIST_CALIB_DURATION}초 동안 주먹을 쥐었다 폈다 반복하세요.")

    # 기간 내에서 IR 값 수집
    elapsed = time.time() - fist_calib_start
    if elapsed < FIST_CALIB_DURATION:
        if ir is not None:
            if ir > MIN_IR_CONTACT:
                fist_ir_list.append(ir)
            print(f"현재 IR 값: {ir}", end='\r')
        return

    # 기간이 끝났으면 threshold 계산
    if len(fist_ir_list) < 100:
        print("\n캘리브레이션 데이터가 너무 적습니다. FIST 캘리브레이션 재시도하세요.")
        # 다시 초기화해서 재시도
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
# [11] RUN 모드: 마우스 움직임 + 클릭
# =========================================================
def run_control(ir, dir_vec):
    global baseline_ir, fist_state, is_mouse_down
    global SCREEN_W, SCREEN_H
    global min_yaw_a, max_yaw_a, min_pitch_a, max_pitch_a

    # -----------------------
    # A. 마우스 움직임 (Z축 방향벡터 -> 각도 -> 화면 좌표)
    # -----------------------
    if min_yaw_a == max_yaw_a or min_pitch_a == max_pitch_a:
        # 캘리브레이션이 안 된 경우
        pass
    else:
        yaw_a, pitch_a = dir_to_angles(dir_vec)

        # 정규화 (0~1)
        normalized_x = (yaw_a - min_yaw_a) / (max_yaw_a - min_yaw_a)
        normalized_y = (pitch_a - min_pitch_a) / (max_pitch_a - min_pitch_a)

        # 필요하면 좌우/상하 반전:
        # normalized_x = 1.0 - normalized_x
        # normalized_y = 1.0 - normalized_y

        normalized_x = np.clip(normalized_x, 0.0, 1.0)
        normalized_y = np.clip(normalized_y, 0.0, 1.0)

        target_x = normalized_x * SCREEN_W
        target_y = normalized_y * SCREEN_H

        mouse.position = (target_x, target_y)

    # -----------------------
    # B. 마우스 클릭 (IR 기반, 기존 로직 유지)
    # -----------------------
    if ir is None:
        # 데이터가 없을 때는 클릭만 안전하게 해제
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
# [12] 시작 안내
# =========================================================
print("===========================================")
print(f"현재 모드: {MODE}")
print("-> 'C' 키를 눌러 캘리브레이션을 시작하세요.")
print("===========================================")
print("### 캘리브레이션 순서: MONITOR(4 꼭짓점) -> FIST(클릭) -> RUN ###")

# =========================================================
# [13] 메인 루프
# =========================================================
while True:
    rate(100)  # VPython 프레임 제한

    pkt = recv_packet()
    if pkt is None:
        # 데이터가 없을 때 RUN 모드에서 클릭이 눌린 채로 멈추지 않게 방지
        if MODE == "RUN" and is_mouse_down:
            mouse.release(Button.left)
            is_mouse_down = False
            fist_state = "IDLE"
        continue

    t_ms, ax, ay, az, dx, dy, dz, ir, yaw, pitch, roll = pkt

    # ----- Z축 방향벡터 & 스무딩 -----
    v = vector(dx, dy, dz)
    if mag(v) < 1e-9:
        # 너무 작은 벡터는 무시
        continue

    new_dir = norm(v)

    # 센서 시간 간격
    if last_t_ms is None:
        dt_ms = 0.0
    else:
        dt_ms = t_ms - last_t_ms
    if dt_ms < 0:
        dt_ms = 0.0
    last_t_ms = t_ms

    # 시간 상수 기반 EMA 계수
    if TAU_MS <= 0:
        alpha = 1.0
    else:
        # dt_ms가 너무 크면 상한을 둬서 과한 점프 방지 (예: 200ms)
        dt_clamped = min(dt_ms, 200.0)
        alpha = 1.0 - np.exp(-dt_clamped / TAU_MS)
    alpha = float(np.clip(alpha, 0.0, 1.0))

    # 작은 각도 변화는 무시 (손 떨림 제거)
    # 기존 filtered_dir가 유효할 때만
    if mag(filtered_dir) > 1e-9:
        dot_val = filtered_dir.x * new_dir.x + filtered_dir.y * new_dir.y + filtered_dir.z * new_dir.z
        dot_val = float(np.clip(dot_val, -1.0, 1.0))
        angle = np.arccos(dot_val)
        if angle < ANGLE_THRESH:
            # 너무 작은 변화 → 업데이트 안 함
            new_dir = filtered_dir

    # 프레임 내에서 여러 번 작은 스텝으로 보간
    if SMOOTH_STEPS <= 1:
        alpha_step = alpha
        filtered_dir = (1.0 - alpha_step) * filtered_dir + alpha_step * new_dir
        if mag(filtered_dir) < 1e-9:
            filtered_dir = new_dir
        else:
            filtered_dir = norm(filtered_dir)
    else:
        alpha_step = alpha / SMOOTH_STEPS
        alpha_step = float(np.clip(alpha_step, 0.0, 1.0))
        for _ in range(SMOOTH_STEPS):
            filtered_dir = (1.0 - alpha_step) * filtered_dir + alpha_step * new_dir
            if mag(filtered_dir) < 1e-9:
                filtered_dir = new_dir
                break
            filtered_dir = norm(filtered_dir)

    # VPython 화살표 업데이트
    z_axis_arrow.axis = filtered_dir * ARROW_LENGTH

    # 정보 텍스트 업데이트
    t_s = t_ms / 1000.0
    info_label.text = (
        f"Sensor time: {t_s:.3f} s  (Δt = {dt_ms:.1f} ms)\n"
        f"EMA alpha: {alpha:.3f}\n"
        f"IR: {ir}"
    )

    # ----- 모드별 동작 -----
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
