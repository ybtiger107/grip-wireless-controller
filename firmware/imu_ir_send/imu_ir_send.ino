#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// (필요하면) PPG용
#include "MAX30105.h" 

// ------------------------------------
// ===== 1. WiFi 및 통신 설정 =====
// ------------------------------------
const char* ssid     = "YB16Pro";     // !!! 자신의 WiFi 이름으로 변경하세요 !!!
const char* password = "12345678";    // !!! 자신의 WiFi 비밀번호로 변경하세요 !!!

// PC IP (반드시 데이터를 수신할 PC의 IP로 맞춰야 합니다!)
const char* udpAddress = "172.20.10.7";  // !!! PC의 실제 IP 주소로 변경하세요 !!!
const int   udpPort    = 9000;

WiFiUDP Udp;

// ------------------------------------
// ===== 2. 센서 객체 및 변수 정의 =====
// ------------------------------------
MPU6050 mpu;
MAX30105 ppg;    // MAX30102도 이 클래스로 사용 가능

// ===== DMP 관련 =====
bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;            // 쿼터니언
float euler[3];          // Yaw, Pitch, Roll을 저장할 float 배열 (라디안)
VectorInt16 aa;          // raw accel (중력 포함)
VectorInt16 aaReal;      // linear accel (중력 제거)
VectorFloat gravity;     // unit gravity vector (~[-1, 1])

// 1g에 해당하는 raw 크기 (LSB) - 자동 보정 대상
float g_lsb = 8192.0f;   // 초기 기본값

// ------------------------------------
// ===== 3. 핵심 기능 함수 =====
// ------------------------------------

// 쿼터니언 → 로컬 Z축 방향벡터 (월드좌표계 기준) 계산
void getZAxisDirectionFromQuaternion(const Quaternion &q, float &zx, float &zy, float &zz) {
  float qw = q.w;
  float qx = q.x;
  float qy = q.y;
  float qz = q.z;

  // 로컬 Z축이 월드에서 향하는 방향 (회전 행렬의 3열)
  zx = 2.0f * (qx * qz + qy * qw);
  zy = 2.0f * (qy * qz - qx * qw);
  zz = 1.0f - 2.0f * (qx * qx + qy * qy);
}

// g_lsb 자동 캘리브레이션 (1g의 raw 크기 측정)
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
    mpu.dmpGetAccel(&aa, fifoBuffer);

    float ax = (float)aa.x;
    float ay = (float)aa.y;
    float az = (float)aa.z;
    float mag = sqrt(ax*ax + ay*ay + az*az);

    sum_mag += mag;
    count++;
    delay(3);
  }

  g_lsb = (float)(sum_mag / (double)N);
  Serial.print("측정된 g_lsb (1g의 raw 크기) = ");
  Serial.println(g_lsb);
}

// ------------------------------------
// ===== 4. setup() 함수 =====
// ------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- WiFi 연결 ---
  Serial.println("=== WiFi 연결 시도 ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nWiFi 연결 성공!");
  Serial.print("D1 mini IP: ");
  Serial.println(WiFi.localIP());

  Udp.begin(udpPort);
  Serial.println("UDP 준비 완료");

  // --- I2C 초기화 (ESP8266: SDA=D2, SCL=D1) ---
  Wire.begin(D2, D1);
  Wire.setClock(400000);

  // --- MPU6050 + DMP 초기화 ---
  Serial.println("MPU6050 DMP 초기화 중...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 연결 실패! 배선/I2C 확인!");
    while (1) delay(1000);
  }

  uint8_t devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    Serial.println("MPU 캘리브레이션 중... (센서를 가만히 두세요)");
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    mpu.setDMPEnabled(true);
    dmpReady   = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    Serial.println("DMP 활성화 완료");
    calibrateG();  // 1g 크기 측정

  } else {
    Serial.print("DMP 초기화 실패, 코드 = ");
    Serial.println(devStatus);
    while (1) delay(1000);
  }

  // --- MAX30102 (선택) 초기화 ---
  if (ppg.begin(Wire, I2C_SPEED_FAST)) {
    ppg.setup();
    ppg.setPulseAmplitudeIR(0x1F);
    ppg.setPulseAmplitudeRed(0x0A);
    ppg.setPulseAmplitudeGreen(0x00);
    Serial.println("MAX30102 초기화 완료");
  } else {
    Serial.println("MAX30102 미검출 (무시하고 진행)");
  }
}

// ------------------------------------
// ===== 5. loop() 함수 (핵심) =====
// ------------------------------------

void loop() {
  if (!dmpReady) return;

  // FIFO 관리 (생략 가능)
  int16_t fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) {
    delay(2);
    return;
  }
  if (fifoCount >= 1024) {
    mpu.resetFIFO();
    return;
  }

  // DMP 패킷 읽기
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // 1. 쿼터니언 및 중력, Raw 가속도 읽기
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetAccel(&aa, fifoBuffer);

  // 2. 오일러 각(Yaw, Pitch, Roll) 계산
  // 배열의 첫 번째 원소 포인터 &euler[0]를 전달하여 float* 인자 타입을 만족시킵니다.
  mpu.dmpGetEuler(&euler[0], &q); 
  
  // 오일러 각을 Degrees(도) 단위로 변환
  float yaw_rad   = euler[0];
  float pitch_rad = euler[1];
  float roll_rad  = euler[2];

  float roll_deg  = roll_rad  * 180 / M_PI;
  float pitch_deg = pitch_rad * 180 / M_PI;
  float yaw_deg   = yaw_rad   * 180 / M_PI;
  
  // 3. 선형 가속도 계산 및 m/s^2 변환
  aaReal.x = aa.x - gravity.x * g_lsb;
  aaReal.y = aa.y - gravity.y * g_lsb;
  aaReal.z = aa.z - gravity.z * g_lsb;

  const float accelScale = 9.80665f / g_lsb;
  float ax = aaReal.x * accelScale;
  float ay = aaReal.y * accelScale;
  float az = aaReal.z * accelScale;

  // 4. 로컬 Z축 방향 벡터 계산
  float zx, zy, zz;
  getZAxisDirectionFromQuaternion(q, zx, zy, zz);

  // 5. IR 값 읽기
  long irValue = ppg.getIR();

  // 6. UDP 패킷 구성 및 전송 (총 11개 필드)
  unsigned long t_ms = millis();
  String payload = String(t_ms) + "," +
                   String(ax, 4) + "," +
                   String(ay, 4) + "," +
                   String(az, 4) + "," +
                   String(zx, 4) + "," +
                   String(zy, 4) + "," +
                   String(zz, 4) + "," +
                   String(irValue) + "," +
                   String(yaw_deg, 2) + "," +
                   String(pitch_deg, 2) + "," +
                   String(roll_deg, 2);

  Udp.beginPacket(udpAddress, udpPort);
  Udp.print(payload);
  Udp.endPacket();

  Serial.println(payload);

  delay(5);
}
