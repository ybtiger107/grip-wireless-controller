#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// (필요하면) PPG용
#include "MAX30105.h"

// ===== WiFi 설정 =====
const char* ssid     = "YB16Pro";
const char* password = "12345678";

// PC IP (반드시 PC IP로 맞춰야 함)
const char* udpAddress = "172.20.10.7";
const int   udpPort    = 9000;

WiFiUDP Udp;

// ===== 센서 객체 =====
MPU6050 mpu;
MAX30105 ppg;   // MAX30102도 이 클래스로 사용

// ===== DMP 관련 =====
bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;           // 쿼터니언
VectorInt16 aa;         // raw accel (중력 포함)
VectorInt16 aaReal;     // 우리가 직접 계산할 linear accel (중력 제거)
VectorFloat gravity;    // unit gravity vector (~[-1, 1])

// 1g에 해당하는 raw 크기 (LSB) - 자동 보정 대상
float g_lsb = 8192.0f;  // 기본값 (DMP 표준), calibrateG()에서 다시 측정

// 쿼터니언 → 로컬 Z축 방향벡터 (월드좌표계 기준) 계산
void getZAxisDirectionFromQuaternion(const Quaternion &q, float &zx, float &zy, float &zz) {
  float qw = q.w;
  float qx = q.x;
  float qy = q.y;
  float qz = q.z;

  // 표준 회전행렬 (local → world)
  // R의 세 번째 컬럼이 로컬 Z축이 월드에서 향하는 방향
  float R13 = 2.0f * (qx * qz + qy * qw);
  float R23 = 2.0f * (qy * qz - qx * qw);
  float R33 = 1.0f - 2.0f * (qx * qx + qy * qy);

  zx = R13;
  zy = R23;
  zz = R33;
}

// ===== g_lsb 자동 캘리브레이션 (1g의 raw 크기 측정) =====
void calibrateG() {
  const int N = 300;    // 샘플 개수
  long count = 0;
  double sum_mag = 0.0;

  Serial.println("g_lsb 캘리브레이션 중... (센서를 가만히 두세요)");

  while (count < N) {
    int16_t fifoCount = mpu.getFIFOCount();
    if (fifoCount < packetSize) {
      delay(2);
      continue;
    }
    if (fifoCount >= 1024) {
      mpu.resetFIFO();
      continue;
    }

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // raw accel만 읽어서 크기 측정
    mpu.dmpGetAccel(&aa, fifoBuffer);   // aa: raw 가속도 (중력 포함)

    float ax = (float)aa.x;
    float ay = (float)aa.y;
    float az = (float)aa.z;
    float mag = sqrt(ax*ax + ay*ay + az*az);  // |a|

    sum_mag += mag;
    count++;
    delay(3);
  }

  g_lsb = (float)(sum_mag / (double)N);
  Serial.print("g_lsb (1g의 raw 크기) = ");
  Serial.println(g_lsb);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== WiFi 연결 시도 ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.println("WiFi 연결 성공!");
  Serial.print("D1 mini IP: ");
  Serial.println(WiFi.localIP());

  Udp.begin(udpPort);
  Serial.println("UDP 준비 완료");

  // ===== I2C 초기화 (ESP8266: SDA=D2, SCL=D1) =====
  Wire.begin(D2, D1);
  Wire.setClock(400000);

  // ===== MPU6050 + DMP =====
  Serial.println("MPU6050 DMP 초기화 중...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 연결 실패! 배선/I2C 확인!");
    while (1) delay(1000);
  }

  uint8_t devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    Serial.println("캘리브레이션 중... (센서를 가만히 두세요)");
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println("캘리브레이션 완료. 오프셋:");
    mpu.PrintActiveOffsets();

    mpu.setDMPEnabled(true);
    dmpReady   = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    Serial.println("DMP 활성화 완료");

    // ===== 여기서 1g 크기를 실제로 측정 =====
    calibrateG();

  } else {
    Serial.print("DMP 초기화 실패, 코드 = ");
    Serial.println(devStatus);
    while (1) delay(1000);
  }

  // ===== MAX30102 (선택) =====
  if (ppg.begin(Wire, I2C_SPEED_FAST)) {
    ppg.setup();
    ppg.setPulseAmplitudeIR(0x3F);
    ppg.setPulseAmplitudeRed(0x0A);
    ppg.setPulseAmplitudeGreen(0x00);
    Serial.println("MAX30102 초기화 완료");
  } else {
    Serial.println("MAX30102 미검출 (무시하고 진행)");
  }
}

void loop() {
  if (!dmpReady) return;

  // FIFO에 패킷이 충분히 쌓였는지 체크
  int16_t fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) {
    delay(2);
    return;
  }
  if (fifoCount >= 1024) {
    // 오버플로우 방지
    mpu.resetFIFO();
    Serial.println("FIFO overflow! 리셋");
    return;
  }

  // DMP 패킷 읽기
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // 쿼터니언, 중력(unit vector), raw accel 읽기
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);   // gravity.x,y,z ≈ [-1,1]
  mpu.dmpGetAccel(&aa, fifoBuffer);  // raw accel (중력 포함, LSB)

  // ===== 우리가 직접 linear accel 계산 (aaReal) =====
  // aaReal = aa - gravity * g_lsb
  aaReal.x = aa.x - gravity.x * g_lsb;
  aaReal.y = aa.y - gravity.y * g_lsb;
  aaReal.z = aa.z - gravity.z * g_lsb;

  // LSB → m/s^2로 변환 (이제 1g = g_lsb임)
  const float accelScale = 9.80665f / g_lsb;
  float ax = aaReal.x * accelScale;
  float ay = aaReal.y * accelScale;
  float az = aaReal.z * accelScale;

  // 로컬 Z축이 세계 좌표계에서 향하는 방향벡터
  float zx, zy, zz;
  getZAxisDirectionFromQuaternion(q, zx, zy, zz);

  // ===== IR 값 읽기 =====
  long irValue = ppg.getIR();   // 없으면 0 또는 작은 값

  // 시간(ms)
  unsigned long t_ms = millis();

  // ===== UDP 패킷 구성: time_ms, ax, ay, az, zx, zy, zz, ir =====
  String payload = String(t_ms) + "," +
                   String(ax, 4) + "," +
                   String(ay, 4) + "," +
                   String(az, 4) + "," +
                   String(zx, 4) + "," +
                   String(zy, 4) + "," +
                   String(zz, 4) + "," +
                   String(irValue);

  Udp.beginPacket(udpAddress, udpPort);
  Udp.print(payload);
  Udp.endPacket();

  // 디버깅 출력
  Serial.println(payload);

  delay(5);  // 대략 200Hz 근처 (상황 봐가며 조절 가능)
}
