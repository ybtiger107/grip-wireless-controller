import socket
import time
from collections import deque

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (3D 활성화용)

# ===== UDP 설정 =====
UDP_IP = "0.0.0.0"
UDP_PORT = 9000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

print(f"UDP 수신 대기중... (포트 {UDP_PORT})")

# ===== 플롯 설정 =====
plt.ion()
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_title("IMU Estimated Position (Very Rough)")

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

# 위치 데이터 (최근 max_points개만 저장)
max_points = 1000
xs = deque(maxlen=max_points)
ys = deque(maxlen=max_points)
zs = deque(maxlen=max_points)

# 초기 위치/속도
pos = [0.0, 0.0, 0.0]  # x, y, z
vel = [0.0, 0.0, 0.0]  # vx, vy, vz

line, = ax.plot([], [], [], marker="o", markersize=2)

# 시간: ✅ "마지막으로 패킷을 받았던 시각"
prev_time = time.time()

# ===== 중력/오프셋 보정용 =====
calib_count = 0
calib_max = 100  # 처음 100 샘플 평균으로 보정
sum_ax = sum_ay = sum_az = 0.0
offset_ax = offset_ay = offset_az = 0.0
calibrated = False

print("처음에는 센서를 가만히 두고, 보정이 끝날 때까지 기다려줘요...")

while True:
    got_new_sample = False  # ✅ 이번 루프에서 새 샘플을 받았는지

    # ===== UDP 패킷 읽기 =====
    try:
        data, addr = sock.recvfrom(1024)
    except BlockingIOError:
        # 데이터 없으면 적분은 하지 않고 플롯만 유지
        pass
    else:
        got_new_sample = True

        # ✅ dt는 "새 패킷을 실제로 받은 순간"에만 계산
        now = time.time()
        dt = now - prev_time
        prev_time = now

        # dt 이상치 방어 (너무 큰 dt는 적분 폭주를 유발)
        if dt <= 0 or dt > 0.1:
            dt = 0.02  # 50 Hz 가정

        msg = data.decode("utf-8", errors="ignore").strip()

        # 기대 형식: "ax,ay,az,gx,gy,gz"  (여기선 앞 3개만 사용)
        parts = msg.split(",")
        if len(parts) >= 3:
            try:
                ax_val = float(parts[0])
                ay_val = float(parts[1])
                az_val = float(parts[2])
            except ValueError:
                print("⚠ 가속도 파싱 실패:", msg)
                ax_val = ay_val = az_val = 0.0

            # ===== 보정 단계 =====
            if not calibrated:
                sum_ax += ax_val
                sum_ay += ay_val
                sum_az += az_val
                calib_count += 1

                if calib_count >= calib_max:
                    offset_ax = sum_ax / calib_max
                    offset_ay = sum_ay / calib_max
                    offset_az = sum_az / calib_max
                    calibrated = True
                    print("보정 완료!")
                    print("offset_ax, offset_ay, offset_az =", offset_ax, offset_ay, offset_az)
                # 보정 중에는 적분/플롯 업데이트용 데이터 추가 안 함
            else:
                # ===== 보정된 가속도 (중력/오프셋 제거: 초기 평균을 빼는 매우 단순한 방식) =====
                ax_corr = ax_val - offset_ax
                ay_corr = ay_val - offset_ay
                az_corr = az_val - offset_az

                # ===== 속도/위치 적분 =====
                vel[0] += ax_corr * dt
                vel[1] += ay_corr * dt
                vel[2] += az_corr * dt

                pos[0] += vel[0] * dt
                pos[1] += vel[1] * dt
                pos[2] += vel[2] * dt

                xs.append(pos[0])
                ys.append(pos[1])
                zs.append(pos[2])

    # ===== 플롯 업데이트 =====
    # 새 샘플이 들어왔고 + 보정 끝난 이후에만 점이 움직이도록 하는 편이 깔끔함
    if xs and ys and zs:
        line.set_data(xs, ys)
        line.set_3d_properties(zs)

        # 축 범위 자동 조정 (조금 여유 있게)
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        min_z, max_z = min(zs), max(zs)

        if max_x == min_x: max_x = min_x + 1
        if max_y == min_y: max_y = min_y + 1
        if max_z == min_z: max_z = min_z + 1

        ax.set_xlim(min_x, max_x)
        ax.set_ylim(min_y, max_y)
        ax.set_zlim(min_z, max_z)

    plt.pause(0.01)
