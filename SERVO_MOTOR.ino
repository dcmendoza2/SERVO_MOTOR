#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// TinkerCAD: https://www.tinkercad.com/things/gvnkK7zD98C

MPU6050 mpu;

int16_t gx, gy, gz;

double valgz;

double gyroRange;

Servo servo0;

void setup() {
  // put your setup code here, to run once:
  servo0.attach(3);
  Wire.begin();
  Serial.begin(115200);
  

  mpu.initialize();
  Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");

  switch(mpu.getFullScaleGyroRange())
  {
    case 0:
    Serial.println("Gyro range: +/- 250 degrees/sec");
    gyroRange = 250;
    break;
    case 1:
    Serial.println("Gyro range: +/- 500 degrees/sec");
    gyroRange = 500;
    break;
    case 2:
    Serial.println("Gyro range: +/- 1000 degrees/sec");
    gyroRange = 1000;
    break;
    case 3:
    Serial.println("Gyro range: +/- 2000 degrees/sec");
    gyroRange = 2000;
    break;
  }
  Serial.println("--------------------");
  mpu.setXGyroOffset(5);
  mpu.setYGyroOffset(54);
  mpu.setZGyroOffset(37);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  mpu.getRotation(&gx, &gy, &gz);
  valgz = map2(gz, -32768, 32768, -gyroRange, gyroRange);
  double angDispZ = integrateZ(valgz);
  if (angDispZ > 90){
    angDispZ = -90;
  }
  if (angDispZ < -90) {
    angDispZ = 90;
  }
  Serial.print(angDispZ);
  servo0.write(90 + angDispZ);
  delay(100);
}


double integrateZ(float x) {
  static float last_x = 0;
  static unsigned long last_t = 0;
  static long Ix = 0;

  unsigned long t = millis();

  long dt = t - last_t;
  last_t = t;

  Ix = Ix + (x + last_x) * dt / 2000.0;
  last_x = x;

  return Ix;
}


double map2(double  x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
