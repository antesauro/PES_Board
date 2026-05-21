#pragma once

#include "Servo.h"

namespace arm_steer {
class ServoModule
{
public:
    ServoModule();

    void initialize();
    void enable();
    void setSteeringAngle(float angle);
    void center();
    void disable();
    void setSpeed(float speed);

private:
    Servo m_servo;
};
}

namespace arm_turn {
class ServoModule
{
public:
    ServoModule();

    void initialize();
    void enable();
    void setSteeringAngle(float angle);
    void center();
    void disable();
    void setSpeed(float speed);

private:
    Servo m_servo;
};
}
