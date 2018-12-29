
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#include "CalLib.h"
#include <EEPROM.h>

#define BLDC_TOP_PINNUM 9
#define BLDC_BOTTOM_PINNUM 10
#define JOYSTICK_RX_PINNUM 8
#define JOYSTICK_TX_PINNUM 7 // 사용 X, 임시로
#define DISPLAY_INTERVAL  50


typedef struct Pid {
  float diffAngle;
  float angle;
  float preAngle;
  int control;
  float kp;
  float ki;
  float kd;
  float p;
  float i;
  float d;
  float err;
  float err_prev;
  float de;
  float dt;
  float final;
} PID;

typedef struct servo {
  const int pinNum;
  int pulse;
  int control;
} SERVO;

RTIMU *imu;
RTFusionRTQF fusion;
RTIMUSettings settings;

SoftwareSerial joystick { JOYSTICK_RX_PINNUM, JOYSTICK_TX_PINNUM };
Servo roll;
Servo pitch;
Servo bl0;
Servo bl1;
SERVO x  = { 11, 700, 0 };    // roll
SERVO y = { 12, 700, 0 };     // pitch
PID yaw = { 0, 0, 0, 0, 1, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// buf1[Aileron], Buf2[Elevator], buf3[Throttle], buf4[Rudder], buf5[Checksum], 나머지는 사용x
int buf[8] = {0, 129, 129, 0, 129, 0, 0, 0}; // 카메라보드로부터 읽어들일 8개의 변수
int Lastbuf[8] = {0, 129, 129, 0, 129, 0, 0,}; // 카메라보드로부터 읽어들 8개의 변수의 이전 값
boolean Loop = true; // Control_UART_Update가 실행될 때 if문으로 딱 한 번만 처리해주기위한 변수

float bl0_setValue = 0;
float bl1_setValue = 0;
int bldc_Control = 0;

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;
unsigned long now;
unsigned long nowJoystick = 0;

void setup() {
  Wire.begin();
  Serial.begin(19200);
  joystick.begin(19200);

  int errcode;
  imu = RTIMU::createIMU(&settings);

  Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0) {
    Serial.print("Failed to init IMU: "); Serial.println(errcode);
  }

  if (imu->getCalibrationValid())
    Serial.println("Using compass calibration");
  else
    Serial.println("No valid compass calibration data");

  lastDisplay = lastRate = millis();
  sampleCount = 0;
  fusion.setSlerpPower(0.02);

  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);

  roll.attach(x.pinNum);
  pitch.attach(y.pinNum);
  bl0.attach(BLDC_TOP_PINNUM, 1000, 2000);
  bl1.attach(BLDC_BOTTOM_PINNUM, 1000, 2000);
  bl0.writeMicroseconds(1000);
  bl1.writeMicroseconds(1000);

  delay(1000);
}
void loop() {

  readIMU();
  readJoystick();

  x.control = map(buf[1], 88, 168, 45, 135);           //  roll 조이스틱 데이터
  y.control = map(buf[2], 88, 168, 45, 135);           //  pitch 조이스틱 데이터
  yaw.control = map(buf[4], 80, 176., -45, 45);        //   yaw 조이스틱 데이터
  bldc_Control = map(buf[3], 0, 255, 0, 875);     //   throttle 조이스틱 데이터
  yaw.diffAngle = yaw.angle - yaw.preAngle;

  //debugDisplay();


  roll.write(x.control);
  pitch.write(y.control);
  setPidBLDC(&yaw);
  yaw.preAngle = yaw.angle;

  if (bldc_Control == 1000) {
    bl0.writeMicroseconds(1000);
    bl1.writeMicroseconds(1000);
    yaw.i = 0;
    yaw.final = 0;
    bl0_setValue = 0;
    bl1_setValue = 0;
  }
  else {
    if (yaw.final > 0) {
      bl0_setValue = -fabs(yaw.final) / 2.;
      bl1_setValue = +fabs(yaw.final) / 2.;
    }
    else {
      bl0_setValue = +fabs(yaw.final) / 2.;
      bl1_setValue = -fabs(yaw.final) / 2.;
    }

    bl0_setValue += bldc_Control;
    bl1_setValue += bldc_Control;
    bl0_setValue = constrain(bl0_setValue, 0, 900);
    bl1_setValue = constrain(bl1_setValue, 0, 900);

    bl0.writeMicroseconds(1100 + bl0_setValue);
    bl1.writeMicroseconds(1100 + bl1_setValue);
  }
}

void readJoystick() {
  if ( joystick.available() < 1) {
    if (millis() - nowJoystick > 3000) {
      buf[1] = 129;
      buf[2] = 129;
      buf[3] = 0;
      buf[4] = 129;
    }
  }
  else {
    nowJoystick = millis();
  }
  // 카메라보드로부터 수신될 때 실행되는 while문
  while (joystick.available()) {
    for (int j = 0; j < 8; j++) {
      buf[j] = joystick.read();
    }
    if (buf[5] != 0) { // checksum의 값이 0과 다를 때 Goto
      for (int i = 0; i < 8; i++) {
        //if (i == 5)
        //  continue;
        buf[i] = Lastbuf[i];
      }
    }

    if (Loop) { // 48번줄을 위한 용도
      for (int i = 0; i < 8; i++) {
        //if (i == 5)
        //  continue;
        Lastbuf[i] = buf[i];
      }
      Loop = false;
    }

    // buf값이 송신측에서 규정한 값의 범위를 벗어날경우 이전의 buf값을 현재 buf값에 대입

    if ((buf[1] > 168) || (buf[1] < 88))
      buf[1] = Lastbuf[1];
    if ((buf[2] > 168) || (buf[2] < 88))
      buf[2] = Lastbuf[2];
    if ((buf[3] > 255) || (buf[3] < 0))
      buf[3] = Lastbuf[3];
    if ((buf[4] > 176) || (buf[4] < 80))
      buf[4] = Lastbuf[4];

  }

  if (!Loop) {
    for (int i = 0; i < 8; i++) {
      //if (i == 5)
      //  continue;
      Lastbuf[i] = buf[i];
    }
  }
}

void readIMU() {
  int loopCount = 1;
  unsigned long delta;
  now = millis();
  while (imu->IMURead()) {
    if (++loopCount >= 10)
      continue;
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    sampleCount++;
    if ((delta = now - lastRate) >= 1000) {
      //Serial.print("Sample rate: "); Serial.print(sampleCount);
      if (imu->IMUGyroBiasValid());
      //  Serial.println(", gyro bias valid");
      //else
      //  Serial.println(", calculating gyro bias");

      sampleCount = 0;
      lastRate = now;
    }

    RTVector3& vec = (RTVector3&)fusion.getFusionPose();

    yaw.angle = vec.z() * RTMATH_RAD_TO_DEGREE;
  }
}

void setPidBLDC(PID * pid) {
  pid->err = yaw.control - yaw.diffAngle;
  pid->de = pid->err - pid->err_prev;
  pid->dt = 0.001;
  pid->p = pid->err * pid->kp;
  pid->i = pid->i + pid->err * pid->dt * pid->ki;
  pid->i = constrain(pid->i, -10, 10);
  pid->d = pid->kd * (pid->de / pid->dt);
  pid->final = pid->p + pid->i + pid->d;
  pid->final = constrain(pid->final, -50, 50);
  pid->err_prev = pid->err;
}


void debugDisplay() {
  /*
    for (int i = 1; i < 5; i++) {
    Serial.print(buf[i]); Serial.print("\t");
    }
    Serial.print("\t\t");*/

  Serial.print("yaw: "); Serial.print(yaw.diffAngle); Serial.print("\t");
  Serial.print("bl0: "); Serial.print(bl0_setValue); Serial.print("\t");
  Serial.print("bl1: "); Serial.print(bl1_setValue); Serial.print("\t");
  Serial.print(x.control); Serial.print("\t");
  Serial.print(y.control); Serial.print("\t");
  Serial.print(yaw.control); Serial.print("\t");
  Serial.println(bldc_Control);
}
