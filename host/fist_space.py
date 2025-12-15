# 주먹 쥐는 것을 5초간 반복해서 보정하는 방법

import socket
import time
import keyboard  # pip install keyboard

UDP_IP = "0.0.0.0"
UDP_PORT = 9000

MIN_IR_CONTACT = 10000   # 이 값보다 작으면 접촉 X로 보고 무시
CALIB_DURATION = 5.0     # 캘리브레이션 시간 (초)

# ==== UDP 소켓 설정 ====
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"UDP 수신 대기중... (포트 {UDP_PORT})")


def recv_one_packet():
    """UDP 패킷 하나 받아서 파싱해 IR 값을 리턴"""
    data, addr = sock.recvfrom(1024)
    msg = data.decode("utf-8", errors="ignore").strip()
    parts = msg.split(",")
    if len(parts) != 11:
        return None, None  # 패킷 이상
    try:
        t_ms = int(parts[0])
        ir = int(parts[7])
        return t_ms, ir
    except ValueError:
        return None, None


def calibrate_ir():
    """5초 동안 주먹 쥐었다 펴기를 반복하게 해서
    baseline, THRESH_HIGH, THRESH_LOW, MIN_GESTURE_INTERVAL를 자동 추정"""
    print("\n=== IR 캘리브레이션 시작 ===")
    print("5초 동안 편하게 주먹을 '쥐었다-폈다' 반복해 주세요.")
    print("준비되면 아무 키나 누르세요.")
    input()

    t0 = time.time()
    ir_list = []
    time_list = []

    # 5초 동안 데이터 수집
    while time.time() - t0 < CALIB_DURATION:
        t_ms, ir = recv_one_packet()
        if t_ms is None:
            continue
        if ir < MIN_IR_CONTACT:
            # 접촉 안 된 상황은 캘리브레이션에 사용하지 않음
            continue

        t_sec = t_ms / 1000.0
        ir_list.append(ir)
        time_list.append(t_sec)

    if len(ir_list) < 10:
        print("캘리브레이션에 데이터가 너무 적습니다. 다시 시도하세요.")
        return None

    # ---- baseline (손 편 상태) / peak (주먹 쥘 때) 추정 ----
    ir_sorted = sorted(ir_list)
    n = len(ir_sorted)

    # 하위 20% 평균 → baseline 쪽
    low_slice = ir_sorted[:max(1, n // 5)]
    baseline = sum(low_slice) / len(low_slice)

    # 상위 20% 평균 → 주먹 쥘 때 쪽
    high_slice = ir_sorted[-max(1, n // 5):]
    peak = sum(high_slice) / len(high_slice)

    delta = max(peak - baseline, 1.0)  # 최소 1 이상

    # THRESH 설정 (baseline 상대)
    THRESH_HIGH = delta * 0.6
    THRESH_LOW = delta * 0.4

    # ---- 제스처 주기(주먹 쥐기 사이 간격) 추정 ----
    # 기준값으로 mid를 잡고, mid를 아래→위로 넘어갈 때를 "주먹 쥐기" 이벤트로 간주
    mid = baseline + delta * 0.5

    crossings = []
    prev_ir = ir_list[0]
    prev_t = time_list[0]

    for t, ir in zip(time_list[1:], ir_list[1:]):
        if prev_ir < mid <= ir:  # 아래에서 위로 교차 (rising edge)
            crossings.append(t)
        prev_ir = ir
        prev_t = t

    if len(crossings) >= 2:
        intervals = [t2 - t1 for t1, t2 in zip(crossings, crossings[1:])]
        intervals_sorted = sorted(intervals)
        median_interval = intervals_sorted[len(intervals_sorted) // 2]
        MIN_GESTURE_INTERVAL = median_interval * 0.6
    else:
        # 주먹 한두 번만 쥐었을 경우: 그냥 대략값
        MIN_GESTURE_INTERVAL = 0.5

    print("\n=== 캘리브레이션 결과 ===")
    print(f"baseline IR ≈ {baseline:.1f}")
    print(f"peak IR ≈ {peak:.1f}")
    print(f"delta ≈ {delta:.1f}")
    print(f"THRESH_HIGH ≈ {THRESH_HIGH:.1f}")
    print(f"THRESH_LOW  ≈ {THRESH_LOW:.1f}")
    print(f"MIN_GESTURE_INTERVAL ≈ {MIN_GESTURE_INTERVAL:.3f} s")
    print("=====================================\n")

    return baseline, THRESH_HIGH, THRESH_LOW, MIN_GESTURE_INTERVAL


# ==== 1) 캘리브레이션 실행 ====
calib_result = calibrate_ir()
if calib_result is None:
    raise SystemExit("캘리브레이션 실패. 다시 실행해 주세요.")

baseline_ir, THRESH_HIGH, THRESH_LOW, MIN_GESTURE_INTERVAL = calib_result

# baseline은 이후에 천천히 업데이트할 수도 있어서
# 여기서는 초기값으로만 사용
ALPHA_BASELINE = 0.01

# ==== 2) 실시간 FIST 제스처 감지 모드 ====

state = "IDLE"
last_trigger_time = 0.0

print("실시간 FIST 인식 시작!")
print("주먹을 쥐었다가 펴면 space 키를 입력합니다.\n")

while True:
    data, addr = sock.recvfrom(1024)
    msg = data.decode("utf-8", errors="ignore").strip()
    parts = msg.split(",")

    if len(parts) != 11:
        continue

    try:
        t_ms = int(parts[0])
        ir = int(parts[7])
    except ValueError:
        continue

    if ir < MIN_IR_CONTACT:
        # 접촉 끊기면 초기화
        state = "IDLE"
        # baseline을 완전히 리셋하고 싶다면 아래 주석 해제
        # baseline_ir = None
        continue

    # baseline 업데이트 (IDLE 상태일 때만)
    if state == "IDLE":
        baseline_ir = (1 - ALPHA_BASELINE) * baseline_ir + ALPHA_BASELINE * ir

    now = time.time()

    # 상태머신
    if state == "IDLE":
        if ir > baseline_ir + THRESH_HIGH:
            state = "FIST"
            print(f"[FIST START] IR={ir}, baseline={baseline_ir:.1f}")

    elif state == "FIST":
        if ir < baseline_ir + THRESH_LOW:
            if now - last_trigger_time > MIN_GESTURE_INTERVAL:
                keyboard.press_and_release("space")
                last_trigger_time = now
                print(f"[FIST GESTURE] SPACE! IR={ir}, baseline={baseline_ir:.1f}")
            else:
                print(f"[FIST IGNORED - TOO FAST] IR={ir}")
            state = "IDLE"

    # 디버깅 보고 싶으면 주석 해제
    # print(f"IR={ir}, baseline={baseline_ir:.1f}, state={state}")
