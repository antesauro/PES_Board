#pragma once

#include "Servo.h"

namespace arm_lenkung {
class ServoModule
{
public:
    ServoModule();

    void initialize();
    void enable();
    void setSteeringAngle(float angle);
    void center();
    void disable();

private:
    Servo m_servo;
};
}

namespace arm_drehkranz {
class ServoModule
{
public:
    ServoModule();

    void initialize();
    void enable();
    void setSteeringAngle(float angle);
    void center();
    void disable();

private:
    Servo m_servo;
};
}
