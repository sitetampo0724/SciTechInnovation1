#include <RPLidar.h>
#include "MecanumDriver.h"

RPLidar lidar;
MecanumDriver mecanum(9, 8, 12, 13, 11, 10, 46, 21);
float distances[360] = { 0 };

void setup() {
  Serial.begin(115200);
  lidar.begin(Serial2);

  mecanum.begin();

  // 等待并启动雷达扫描
  delay(500);
  Serial.println("Starting RPLidar...");
  
  rplidar_response_device_info_t info;
  if (IS_OK(lidar.getDeviceInfo(info, 100))) {
    lidar.startScan();
    Serial.println("RPLidar scan started!");
  } else {
    Serial.println("Failed to get device info, check connection!");
  }
}

void loop() {
  if (!lidar.isOpen()) {
    Serial.println("RPLidar not responding, reinitializing...");
    lidar.end();
    delay(100);
    lidar.begin(Serial2);
    delay(500);
    lidar.startScan();
    delay(500);
    return;
  }

  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance / 1000.0;
    int angle = lidar.getCurrentPoint().angle;
    bool startBit = lidar.getCurrentPoint().startBit;

    if (angle >= 0 && angle < 360) {
      distances[angle] = distance;
    }

    if (startBit) {
      float distance_min = 10;
      int angle_min = 0;
      
      for (int i = 0; i < 360; i++) {
        float d = distances[i];
        if (d >= 0.15) {
          if (d < distance_min) {
            distance_min = d;
            angle_min = i;
          }
        }
      }
      
      if (distance_min > 0.5) {
        mecanum.driveAllMotor(0, 0, 0, 0);
      } else {
        if (angle_min >= 337.5 || angle_min < 22.5) mecanum.driveAllMotor(100, 100, 100, 100);
        else if (angle_min >= 22.5 && angle_min < 67.5) mecanum.driveAllMotor(100, 0, 0, 100);
        else if (angle_min >= 67.5 && angle_min < 112.5) mecanum.driveAllMotor(100, -100, -100, 100);
        else if (angle_min >= 112.5 && angle_min < 157.5) mecanum.driveAllMotor(0, -100, -100, 0);
        else if (angle_min >= 157.5 && angle_min < 202.5) mecanum.driveAllMotor(-100, -100, -100, -100);
        else if (angle_min >= 202.5 && angle_min < 247.5) mecanum.driveAllMotor(-100, 0, 0, -100);
        else if (angle_min >= 247.5 && angle_min < 292.5) mecanum.driveAllMotor(-100, 100, 100, -100);
        else if (angle_min >= 292.5 && angle_min < 337.5) mecanum.driveAllMotor(0, 100, 100, 0);
      }
    }
  } else {
    Serial.println("Waiting for RPLidar data...");
    delay(500);
    lidar.end();
    delay(100);
    lidar.begin(Serial2);
    delay(500);
    lidar.startScan();
    delay(500);
  }
}