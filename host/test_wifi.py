# Wifi로부터 받는 UDP 패킷을 확인하는 간단한 스크립트

import socket
import keyboard  # pip install keyboard
import time

UDP_IP = "0.0.0.0"   # 모든 인터페이스에서 받기
UDP_PORT = 9000      # D1 mini 코드의 udpPort와 같게

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"UDP 수신 대기중... (포트 {UDP_PORT})")

now0 = time.time()
while True:
    data, addr = sock.recvfrom(1024)
    now = time.time() - now0
    msg = data.decode("utf-8", errors="ignore").strip()
    print("보낸 쪽:", addr, "/ 시간:", f"{now:.4f}", "/ 내용:", msg)
