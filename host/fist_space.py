# 주먹 쥠 3번 이후 캘리브레이션 시작,
# 캘리브레이션 후 주먹 쥐었다 폈다 하면 스페이스바 입력

import socket
import time
import keyboard  # pip install keyboard

# ===== UDP 수신 설정 =====
UDP_IP = "0.0.0.0"
UDP_PORT = 9000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"UDP 수신 대기중... (포트 {UDP_PORT})")

# ===== 공통 상수 =====
MIN_IR_CONTACT = 10000   # 이 값 미만이면 센서 접촉 X로 보고 무시

# ===== 트리플 FIST 트리거용 파라미터 (대략적인 값) =====
PRE_ALPHA_BASELINE = 0.01    
PRE_THRESH_HIGH = 2000.0
PRE_THRESH_LOW = 1000.0
PRE_MIN_GESTURE_INTERVAL = 0.2  # 너무 빠른 중복 인식 방지
TRIGGER_COUNT = 3
TRIGGER_WINDOW = 2.5            # 이 시간 안에 FIST 3번 → 캘리 시작

# ===== 캘리브레이션 설정 =====
CALIB_DURATION = 7.0            # 5초 동안 IR 수집해서 분석

# ===== 상태 =====
mode = "WAIT_TRIGGER"           # "WAIT_TRIGGER" / "RUN"
pre_state = "IDLE"
pre_baseline_ir = None
pre_last_trigger_time = 0.0
trigger_times = []              # 트리거용 FIST 완료 시간들

# 실전 모드용 상태
state = "IDLE"
baseline_ir = None
ALPHA_BASELINE = 0.01           # 실전 모드에서 baseline 추적 속도
THRESH_HIGH = 1500.0            # 캘리 후 덮어씀
THRESH_LOW = 800.0
MIN_GESTURE_INTERVAL = 0.5
last_trigger_time = 0.0         # 마지막 space 입력 시간


def recv_one_packet():
    """UDP 패킷 하나 받고 (t_ms, ir) 반환. 실패하면 (None, None)"""
    data, addr = sock.recvfrom(1024)
    msg = data.decode("utf-8", errors="ignore").strip()
    parts = msg.split(",")
    if len(parts) != 11:
        return None, None
    try:
        t_ms = int(parts[0])
        ir = int(parts[7])
        return t_ms, ir
    except ValueError:
        return None, None


def calibrate_ir():
    """5초 동안 IR 수집해서 baseline / peak / THRESH / MIN_INTERVAL 추정"""
    print("\n=== IR 캘리브레이션 시작 ===")
    print("지금부터 약 5초 동안 주먹을 쥐었다-폈다를 몇 번 반복해 주세요.\n")

    t0 = time.time()
    ir_list = []
    t_list = []

    while time.time() - t0 < CALIB_DURATION:
        t_ms, ir = recv_one_packet()
        if t_ms is None:
            continue
        if ir < MIN_IR_CONTACT:
            # 접촉 안 된 구간은 사용하지 않음
            continue

        t_sec = t_ms / 1000.0
        ir_list.append(ir)
        t_list.append(t_sec)

    if len(ir_list) < 10:
        print("캘리브레이션 데이터가 너무 적습니다. 다시 시도해주세요.")
        return None

    ir_sorted = sorted(ir_list)
    n = len(ir_sorted)

    # baseline: 하위 20% 평균
    low_slice = ir_sorted[:max(1, n // 5)]
    baseline = sum(low_slice) / len(low_slice)

    # peak: 상위 20% 평균
    high_slice = ir_sorted[-max(1, n // 5):]
    peak = sum(high_slice) / len(high_slice)

    delta = max(peak - baseline, 1.0)

    # threshold는 delta 비율로 설정
    TH = delta * 0.6
    TL = delta * 0.4

    # 제스처 주기 추정
    mid = baseline + delta * 0.5
    crossings = []
    prev_ir = ir_list[0]
    prev_t = t_list[0]

    for t, ir in zip(t_list[1:], ir_list[1:]):
        if prev_ir < mid <= ir:  # 아래→위 교차 (주먹 쥐기 시작 근사)
            crossings.append(t)
        prev_ir = ir
        prev_t = t

    if len(crossings) >= 2:
        intervals = [t2 - t1 for t1, t2 in zip(crossings, crossings[1:])]
        intervals_sorted = sorted(intervals)
        median_interval = intervals_sorted[len(intervals_sorted) // 2]
        min_interval = median_interval * 0.6
    else:
        min_interval = 0.5  # 데이터가 적으면 기본값

    print("=== 캘리브레이션 완료 ===")
    print(f"baseline IR ≈ {baseline:.1f}")
    print(f"peak IR ≈ {peak:.1f}")
    print(f"delta ≈ {delta:.1f}")
    print(f"THRESH_HIGH ≈ {TH:.1f}")
    print(f"THRESH_LOW  ≈ {TL:.1f}")
    print(f"MIN_GESTURE_INTERVAL ≈ {min_interval:.3f} s")
    print("==========================\n")

    return baseline, TH, TL, min_interval


print("모드: WAIT_TRIGGER")
print(f"센서 착용 후, 빠르게 주먹을 {TRIGGER_COUNT}번 쥐었다 폈다 하면 캘리브레이션을 시작합니다.\n")

while True:
    if mode == "WAIT_TRIGGER":
        # === 트리거 대기 모드: 대략적인 FSM으로 FIST 3회 감지 ===
        t_ms, ir = recv_one_packet()
        if t_ms is None:
            continue

        if ir < MIN_IR_CONTACT:
            # 접촉 끊긴 경우
            pre_state = "IDLE"
            pre_baseline_ir = None
            continue

        # baseline 갱신
        if pre_baseline_ir is None:
            pre_baseline_ir = ir
        elif pre_state == "IDLE":
            pre_baseline_ir = (1 - PRE_ALPHA_BASELINE) * pre_baseline_ir + PRE_ALPHA_BASELINE * ir

        now = time.time()

        # 간단 FSM
        if pre_state == "IDLE":
            if ir > pre_baseline_ir + PRE_THRESH_HIGH:
                pre_state = "FIST"
                # print(f"[PRE FIST START] IR={ir}, baseline={pre_baseline_ir:.1f}")
        elif pre_state == "FIST":
            if ir < pre_baseline_ir + PRE_THRESH_LOW:
                # 한 번의 FIST 완료
                if now - pre_last_trigger_time > PRE_MIN_GESTURE_INTERVAL:
                    pre_last_trigger_time = now
                    trigger_times.append(now)
                    # 오래된 트리거 제거
                    trigger_times = [t for t in trigger_times if now - t <= TRIGGER_WINDOW]
                    print(f"[PRE FIST] count={len(trigger_times)} IR={ir}")
                pre_state = "IDLE"

        # 3회 이상 감지되면 캘리 시작
        if len(trigger_times) >= TRIGGER_COUNT:
            print("\n>>> 트리플 FIST 감지! IR 캘리브레이션을 시작합니다.")
            calib = calibrate_ir()
            trigger_times.clear()

            if calib is None:
                print("캘리브레이션 실패. 다시 트리플 FIST를 해주세요.\n")
                continue

            baseline_ir, THRESH_HIGH, THRESH_LOW, MIN_GESTURE_INTERVAL = calib
            state = "IDLE"
            last_trigger_time = 0.0

            mode = "RUN"
            print("모드: RUN (주먹 쥐었다-폈다 → space 입력)\n")

    elif mode == "RUN":
        # === 실전 모드: 캘리한 값을 사용해 FIST → space ===
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
            state = "IDLE"
            continue

        # baseline 갱신 (IDLE 상태에서만)
        if baseline_ir is None:
            baseline_ir = ir
        elif state == "IDLE":
            baseline_ir = (1 - ALPHA_BASELINE) * baseline_ir + ALPHA_BASELINE * ir

        now = time.time()

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

        # 디버깅 원하면:
        # print(f"IR={ir}, baseline={baseline_ir:.1f}, state={state}")
