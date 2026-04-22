#include "motor_module_Arm.h"

#include "PESBoardPinMap.h"

#include <cmath>

MotorModuleArm::MotorModuleArm() :
    m_enableMotors(PB_ENABLE_DCMOTORS),
    m_motor(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, GEAR_RATIO, KN, VOLTAGE_MAX)
{
    initialize();
}

void MotorModuleArm::initialize()
{
    enableMotors();
    m_motor.enableMotionPlanner();
    const float current_rotations = m_motor.getRotation();
    m_motor.setMotionPlannerVelocity(0.0f);
    m_motor.setMotionPlannerPosition(current_rotations);
    m_initialized = true;
}

void MotorModuleArm::set(float rotations)
{
    ensureReadyForCommand();
    m_motor.setRotation(rotations);
}

void MotorModuleArm::setVelocity(float velocity_rps)
{
    ensureReadyForCommand();
    m_motor.setVelocity(velocity_rps);
}

float MotorModuleArm::get() const
{
    return m_motor.getRotation();
}

bool MotorModuleArm::setAndWait(float rotations, float tolerance, int timeout_ms)
{
    ensureReadyForCommand();
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

void MotorModuleArm::enableMotors()
{
    m_enableMotors = 1;
    m_motor.enableMotionPlanner();
}

void MotorModuleArm::disableMotors()
{
    m_motor.disableMotionPlanner();
    m_enableMotors = 0;
}

void MotorModuleArm::ensureReadyForCommand()
{
    if (!m_initialized) {
        initialize();
        return;
    }

    enableMotors();
}