#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "MAX30105.h"   // MAX30102용 라이브러리

Adafruit_MPU6050 mpu;
MAX30105 ppg;   // MAX30102도 MAX30105 클래스로 사용

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ESP8266 I2C 초기화: (SDA, SCL)
  Wire.begin(D2, D1);

  // ===== MPU6050 초기화 =====
  if (!mpu.begin(0x68)) {   // I2C 스캐너에서 주소 확인 후 필요시 수정
    Serial.println("MPU6050를 찾지 못했습니다. 배선/I2C 주소 확인!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("MPU6050 초기화 완료");

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // ===== MAX30102 초기화 =====
  if (!ppg.begin(Wire, I2C_SPEED_FAST)) {  // 기본 주소 0x57
    Serial.println("MAX30102를 찾지 못했습니다. 배선/I2C 주소 확인!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("MAX30102 초기화 완료");

  // 심박 측정용 기본 설정
  ppg.setup();                   // 기본 설정값 사용
  ppg.setPulseAmplitudeRed(0x0A); // Red LED (SpO2 등 쓸 때)
  ppg.setPulseAmplitudeIR(0x3F);  // IR LED 세기 (맥파용)
  ppg.setPulseAmplitudeGreen(0x00); // Green LED 사용 안 함
}

void loop() {
  // ===== MPU6050 데이터 읽기 =====
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // ===== MAX30102 데이터 읽기 =====
  long irValue  = ppg.getIR();   // IR 채널 (맥파)
  long redValue = ppg.getRed();  // Red 채널 (필요시 사용)

  // ===== Serial Plotter용 출력 형식 =====
  // "label value" 형식으로 여러 개 한 줄에 출력하면
  // Arduino Serial Plotter에서 채널별로 그래프가 나뉨.

  Serial.print("accX:");
  Serial.print(accel.acceleration.x);
  Serial.print(" accY:");
  Serial.print(accel.acceleration.y);
  Serial.print(" accZ:");
  Serial.print(accel.acceleration.z);

  Serial.print(" gyroX:");
  Serial.print(gyro.gyro.x);
  Serial.print(" gyroY:");
  Serial.print(gyro.gyro.y);
  Serial.print(" gyroZ:");
  Serial.print(gyro.gyro.z);

  Serial.print(" ir:");
  Serial.print(irValue);
  Serial.print(" red:");
  Serial.print(redValue);

  Serial.println(); // 한 줄 끝

  delay(20);  // ≈ 50Hz 출력 (너무 빨라서 WDT 걸리는 것 방지)
}

