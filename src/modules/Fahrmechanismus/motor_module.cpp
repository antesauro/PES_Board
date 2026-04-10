#include "motor_module.h"

#include "PESBoardPinMap.h"

MotorModule::MotorModule()
    : m_enableMotors(PB_ENABLE_DCMOTORS)
    , m_motor(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, GEAR_RATIO, KN, VOLTAGE_MAX)
{
    m_enableMotors = 0;
}

void MotorModule::initialize()
{
    m_enableMotors = 1;
    m_motor.setMotionPlannerVelocity(0.0f);
    m_motor.setMotionPlannerPosition(0.0f);
    m_motor.enableMotionPlanner();
}

void MotorModule::setVelocity(float velocity_rps) { m_motor.setVelocity(velocity_rps); }

void MotorModule::setRotation(float rotations) { m_motor.setRotation(rotations); }

float MotorModule::getVelocity() const { return m_motor.getVelocity(); }

float MotorModule::getRotation() const { return m_motor.getRotation(); }

void MotorModule::stop() { m_motor.setVelocity(0.0f); }

void MotorModule::disable()
{
    stop();
    m_enableMotors = 0;
}
