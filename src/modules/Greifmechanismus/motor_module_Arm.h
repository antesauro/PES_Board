#pragma once

#include "DCMotor.h"
#include "mbed.h"

class MotorModuleArm
{
public:
    MotorModuleArm();

    // Set target motor position in rotations.
    void set(float rotations);

    // Set target rotation and block until reached (or timeout).
    bool setAndWait(float rotations, float tolerance = 0.03f, int timeout_ms = 3000);

private:
    static constexpr float GEAR_RATIO  = 78.125f;
    static constexpr float KN          = 180.0f / 12.0f; // rpm/V
    static constexpr float VOLTAGE_MAX = 12.0f;

    DigitalOut m_enableMotors;
    DCMotor m_motor;
};