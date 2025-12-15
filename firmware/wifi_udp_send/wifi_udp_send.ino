#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* ssid     = "YB16Pro";   // 네가 실제 쓰는 핫스팟 SSID
const char* password = "12345678";  // 실제 비번

// === 여기 "PC IP 주소"로 바꿔야 함 ===
const char* udpAddress = "172.20.10.7"; // 예시: 아이폰 핫스팟이면 보통 172.20.10.1이 폰/테더링 게이트웨이
const int   udpPort    = 9000;          // PC에서 받을 포트 번호

WiFiUDP Udp;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("WiFi 연결 시도 중...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println();
  Serial.println("WiFi 연결 성공!");
  Serial.print("D1 mini IP: ");
  Serial.println(WiFi.localIP());

  Udp.begin(udpPort);
  Serial.println("UDP 준비 완료");
}

void loop() {
  const char* msg = "HELLO_FROM_D1_MINI";

  Udp.beginPacket(udpAddress, udpPort);
  Udp.write((const uint8_t*)msg, strlen(msg));
  Udp.endPacket();

  Serial.println("UDP 패킷 전송: HELLO_FROM_D1_MINI");

  delay(1000); // 1초마다 전송
}
