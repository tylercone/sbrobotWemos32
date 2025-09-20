#ifndef BALANCE_H
#define BALANCE_H

#include <Arduino.h>
#include "gyro/gyro.h"

// PID controller structure
struct PIDController
{
    float kp; // Proportional gain
    float ki; // Integral gain
    float kd; // Derivative gain
    float integral;
    float previousError;
    unsigned long lastTime;
    int baseSpeed;
};

// Global variables
extern GyroOffsets gyroOffsets;
extern AccelOffsets accelOffsets;

// Function declarations
void initBalance();
float updatePID(PIDController &pid, float error);
void balanceRobot(float angle, float deadBand);
void setPrincipal(float gain);
void setIntegral(float gain);
void setDerivitive(float gain);
void setBaseSpeed(float gain);
void adjustPIDGains(char input);

// Global PID controller access
extern PIDController balancePID;

#endif
