#pragma once

#include "Servo.h"

class ServoModule
{
public:
    ServoModule();

    void initialize();
    // Set the normalized steering angle: 0.0 = full left, 0.5 = center, 1.0 = full right.
    // This is a position command — NOT a PWM duty cycle like a DC motor.
    void setSteeringAngle(float angle);
    void center();
    void disable();

private:
    Servo m_servoD0;
};
