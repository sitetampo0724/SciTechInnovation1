#include <RPLidar.h>
#include "MecanumDriver.h"

RPLidar lidar;
MecanumDriver mecanum(9, 8, 12, 13, 11, 10, 46, 21);
float distances[360] = { 0 };

// ================== SmartRotate 校正参数 ==================
const float KP         = 75.0f;
const float KD         = 350.0f;
const float STEER_MAX  = 60.0f;
const float BASE_SPEED = 100.0f;
const float DEAD_ZONE  = 0.04f;

int count=0;

float prevError = 0.0f;

// ================== 采样函数 ==================
float weightedMean(float a[], int from, int to) {
    float sumW = 0, sumD = 0;
    for (int i = from; i <= to; i++) {
        if (a[i] < 0.05f) continue;
        float w = 1.0f / (a[i] + 0.1f);
        sumW += w;
        sumD += a[i] * w;
    }
    return (sumW > 0) ? (sumD / sumW) : 9999.0f;
}

float getAverageDist(float a[], int from, int to) {
    float sum = 0; int count = 0;
    for (int i = from; i <= to; i++) {
        if (a[i] > 0.05f) { sum += a[i]; count++; }
    }
    return (count > 0) ? (sum / count) : 9999.0f;
}

float getFrontDistance(float a[]) { return getAverageDist(a, 175, 185); }
float getLeftDistance(float a[])  { return getAverageDist(a,  85,  95); }
float getRightDistance(float a[]) { return getAverageDist(a, 265, 275); }

// ================== 增强版 SmartRotate (PD控制) ==================
void SmartRotate(float a[]) {
    float leftDist  = weightedMean(a,  40, 140);
    float rightDist = weightedMean(a, 220, 320);

    float error = leftDist - rightDist;
    if (fabsf(error) < DEAD_ZONE) error = 0;

    // 微分项：偏差收缩时产生反向阻尼，防止过冲
    float dError = error - prevError;
    prevError = error;

    float steer = error * KP + dError * KD;
    steer = constrain(steer, -STEER_MAX, STEER_MAX);

    // 前向速度随偏差动态衰减
    float fwd = BASE_SPEED * (1.0f - fabsf(steer) / (STEER_MAX * 1.5f));
    fwd = constrain(fwd, 40.0f, BASE_SPEED);

    int leftSpeed  = constrain((int)(fwd - steer), -120, 120);
    int rightSpeed = constrain((int)(fwd + steer), -120, 120);

    mecanum.driveAllMotor(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
}

// ================== 状态机 ==================
enum RobotState { NORMAL, TURNING_LEFT, TURNING_RIGHT, TURN_PAUSE, TURNING_LEFT_Intersection, TURNING_RIGHT_Intersection };
RobotState currentState = NORMAL;
unsigned long stateStartTime = 0;

// ================== 左右转弯独立时长参数 ==================
const unsigned long TURN_DURATION_LEFT  = 745;   // 左转时长
const unsigned long TURN_DURATION_RIGHT = 765;   // 右转时长
const unsigned long PAUSE_DURATION = 500;
const unsigned long TURN_DURATION_Intersection = 615;

void setup() {
    Serial.begin(115200);
    lidar.begin(Serial2);
    mecanum.begin();
    delay(1000);
}

void loop() {
    if (IS_OK(lidar.waitPoint())) {
        float distance = lidar.getCurrentPoint().distance / 1000.0f;
        int   angle    = (int)lidar.getCurrentPoint().angle;
        bool  startBit = lidar.getCurrentPoint().startBit;

        if (angle >= 0 && angle < 360)
            distances[angle] = distance;
        
        if (startBit) {
            
            float frontDist = getFrontDistance(distances);
            float leftDist  = getLeftDistance(distances);
            float rightDist = getRightDistance(distances);

            switch (currentState) {
                case NORMAL:
                    SmartRotate(distances);
                    if(count==3&&leftDist>0.5f)
                    {
                        currentState = TURNING_LEFT_Intersection;
                        stateStartTime = millis();
                        mecanum.driveAllMotor(-60, 100, -60, 100);
                        count+=1;
                    }
                    if(count==7&&rightDist>0.5f)
                    {
                        currentState = TURNING_RIGHT_Intersection;
                        stateStartTime = millis();
                        mecanum.driveAllMotor(100, -60, 100, -60);
                        count=0;
                    }
                    if (frontDist < 0.75f) {
                       
                        if (count%7<=3 && leftDist > 0.7f) {
                            currentState = TURNING_LEFT;
                            stateStartTime = millis();
                            mecanum.driveAllMotor(-50, 120, -50, 120);
                            count+=1;
                        } else if (count%7>=4&&rightDist > 0.8f) {
                            currentState = TURNING_RIGHT_Intersection;
                            stateStartTime = millis();
                            mecanum.driveAllMotor(100, -50, 100, -50);
                            count+=1;
                        }
                    }
                    break;

                case TURNING_LEFT:
                    if (millis() - stateStartTime >= TURN_DURATION_LEFT) {
                        currentState = TURN_PAUSE;
                        stateStartTime = millis();
                        mecanum.driveAllMotor(100, 100, 100, 100);
                    }
                    break;

                case TURNING_RIGHT:
                    if (millis() - stateStartTime >= TURN_DURATION_RIGHT) {
                        currentState = TURN_PAUSE;
                        stateStartTime = millis();
                        mecanum.driveAllMotor(100, 100, 100, 100);
                    }
                    break;

                case TURNING_LEFT_Intersection:
                    if (millis() - stateStartTime >= TURN_DURATION_Intersection) {
                        currentState = TURN_PAUSE;
                        stateStartTime = millis();
                        mecanum.driveAllMotor(100, 100, 100, 100);
                    }
                    break;

                case TURNING_RIGHT_Intersection:
                    if (millis() - stateStartTime >= TURN_DURATION_Intersection) {
                        currentState = TURN_PAUSE;
                        stateStartTime = millis();
                        mecanum.driveAllMotor(100, 100, 100, 100);
                    }
                    break;

                case TURN_PAUSE:
                    if (millis() - stateStartTime >= PAUSE_DURATION) {
                        prevError = 0.0f;  // 清除转弯残留微分
                        currentState = NORMAL;
                    }
                    break;
            }
        }
    } else {
        rplidar_response_device_info_t info;
        if (IS_OK(lidar.getDeviceInfo(info, 100))) {
            lidar.startScan();
            delay(1000);
        }
    }
}