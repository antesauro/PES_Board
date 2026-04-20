#include "motor_module_Arm.h"

#include "PESBoardPinMap.h"

#include <cmath>

MotorModuleArm::MotorModuleArm() :
    m_enableMotors(PB_ENABLE_DCMOTORS),
    m_motor(PB_PWM_M3, PB_ENC_A_M3, PB_ENC_B_M3, GEAR_RATIO, KN, VOLTAGE_MAX)
{
    m_enableMotors = 1;
    m_motor.enableMotionPlanner();
}

void MotorModuleArm::set(float rotations)
{
    m_motor.setRotation(rotations);
}

void MotorModuleArm::setVelocity(float velocity_rps)
{
    m_motor.setVelocity(velocity_rps);
}

float MotorModuleArm::get() const
{
    return m_motor.getRotation();
}

bool MotorModuleArm::setAndWait(float rotations, float tolerance, int timeout_ms)
{
    m_motor.setRotation(rotations);

    Timer timer;
    timer.start();

    while (true) {
        const float error = std::fabs(rotations - m_motor.getRotation());
        if (error <= tolerance) {
            return true;
        }

        if (std::chrono::duration_cast<std::chrono::milliseconds>(timer.elapsed_time()).count() >= timeout_ms) {
            return false;
        }

        thread_sleep_for(10);
    }
}