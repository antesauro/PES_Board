#pragma once

#include "DCMotor.h"
#include "mbed.h"

class MotorModuleArm
{
public:
    MotorModuleArm();

    // Set target motor position in rotations.
    void set(float rotations);

    // Command motor velocity in rotations per second.
    void setVelocity(float velocity_rps);

    // Read current motor position in rotations.
    float get() const;

    // Set target rotation and block until reached (or timeout).
    bool setAndWait(float rotations, float tolerance = 0.03f, int timeout_ms = 3000);

    // Motor enable/disable
    void enableMotors() { m_enableMotors = 1; }
    void disableMotors() { m_enableMotors = 0; }

private:
    static constexpr float GEAR_RATIO  = 78.125f;
    static constexpr float KN          = 180.0f / 12.0f; // rpm/V
    static constexpr float VOLTAGE_MAX = 12.0f;

    DigitalOut m_enableMotors;
    DCMotor m_motor;
};