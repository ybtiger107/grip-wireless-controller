import socket
import time
import keyboard  # pip install keyboard

UDP_IP = "0.0.0.0"
UDP_PORT = 9000

# ==== IR 제스처 감지 파라미터 ====
MIN_IR_CONTACT = 10000   # 이 값보다 작으면 접촉 X로 보고 무시
ALPHA_BASELINE = 0.01    # baseline 업데이트 속도 (0~1, 작을수록 느리게 따라감)
THRESH_HIGH = 3000       # baseline보다 이만큼 ↑ 올라가면 "주먹 쥠" 시작
THRESH_LOW = 3000         # 다시 baseline + THRESH_LOW 아래로 내려오면 "주먹 풀림"으로 간주
MIN_GESTURE_INTERVAL = 0.5  # (초) 너무 자주 트리거되지 않도록 최소 간격

# 상태
state = "IDLE"           # "IDLE" 또는 "FIST"
baseline_ir = None       # 손이 편-안 상태일 때의 IR 평균
last_trigger_time = 0.0  # 마지막으로 space 입력한 시간

# ==== UDP 소켓 설정 ====
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"UDP 수신 대기중... (포트 {UDP_PORT})")
print("주먹 쥐었다가 펴면 space 키를 누릅니다. (IR >= 10000 기준)")

while True:
    data, addr = sock.recvfrom(1024)
    msg = data.decode("utf-8", errors="ignore").strip()

    parts = msg.split(",")
    if len(parts) != 11:
        print("패킷 형식 오류:", msg)
        continue

    try:
        t_ms = int(parts[0])
        ax = float(parts[1])
        ay = float(parts[2])
        az = float(parts[3])
        zx = float(parts[4])
        zy = float(parts[5])
        zz = float(parts[6])
        ir = int(parts[7])
    except ValueError:
        print("숫자 파싱 실패:", msg)
        continue

    # ==== IR 사용 조건: 최소 값 이상일 때만 사용 ====
    if ir < MIN_IR_CONTACT:
        # 접촉이 끊긴 걸로 보고 상태 리셋
        baseline_ir = None
        state = "IDLE"
        # 디버그 보고 싶으면 주석 해제
        # print(f"[NO CONTACT] IR={ir}")
        continue

    # ==== baseline 업데이트 (주먹 안 쥐었을 때만) ====
    if baseline_ir is None:
        baseline_ir = ir  # 초기화
    else:
        if state == "IDLE":
            # 손이 대충 편안 상태일 때 baseline 천천히 추적
            baseline_ir = (1 - ALPHA_BASELINE) * baseline_ir + ALPHA_BASELINE * ir

    now = time.time()

    # ==== 상태머신 ====
    if state == "IDLE":
        # 손이 편-안한 상태라고 가정
        # 기준보다 충분히 많이 올라가면 "주먹 쥐기" 시작
        if ir > baseline_ir + THRESH_HIGH:
            state = "FIST"
            print(f"[FIST START] IR={ir}, baseline={baseline_ir:.1f}")

    elif state == "FIST":
        # 다시 baseline 근처로 내려오면 "주먹 풀림"으로 간주
        if ir < baseline_ir + THRESH_LOW:
            # 제스처 한 사이클 완료 (쥐었다 → 풀었다)
            if now - last_trigger_time > MIN_GESTURE_INTERVAL:
                keyboard.press_and_release("space")
                last_trigger_time = now
                print(f"[FIST GESTURE] SPACE! IR={ir}, baseline={baseline_ir:.1f}")
            else:
                print(f"[FIST GESTURE IGNORED - TOO FAST] IR={ir}")
            state = "IDLE"

    # 디버깅 출력 (원하면 살려서 추세 보기)
    # print(f"time={t_ms}, IR={ir}, baseline={baseline_ir:.1f} state={state}")
