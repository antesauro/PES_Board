#pragma once

#include "DCMotor.h"
#include "mbed.h"

/**
 * @brief Wrapper around the DCMotor library for the PES-board drive motor (M1).
 *
 * DCMotor uses encoder feedback and an internal PID + motion planner to
 * regulate velocity (rps) or position (rotations). This is fundamentally
 * different from a raw PWM duty-cycle: the library converts the commanded
 * value to a PWM output automatically.
 *
 * Physical parameters for the motor on M1:
 *   gear_ratio = 78.125
 *   kn         = 180 rpm/V  (at 12 V supply)
 *   voltage_max = 12 V
 */
class MotorModule
{
public:
    MotorModule();

    // Enable the motor driver and reset internal state.
    void initialize();

    // Command a target velocity in rotations per second.
    void setVelocity(float velocity_rps);

    // Command a target absolute position in rotations.
    void setRotation(float rotations);

    // Read back current velocity (rps) and position (rotations).
    float getVelocity() const;
    float getRotation() const;

    // Gently decelerate to zero (uses motion planner).
    void stop();

    // Disable the motor driver power stage.
    void disable();

private:
// Physical parameters for the motor on M1:
    static constexpr float GEAR_RATIO   = 78.125f;
    static constexpr float KN           = 180.0f / 12.0f; // rpm/V
    static constexpr float VOLTAGE_MAX  = 12.0f;

    DigitalOut m_enableMotors;
    DCMotor    m_motor;
};
