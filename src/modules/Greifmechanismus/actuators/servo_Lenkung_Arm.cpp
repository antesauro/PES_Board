#include "servo_module_Arm.h"

#include "PESBoardPinMap.h"

namespace arm_lenkung {
namespace {
// Physical pulse width limits (seconds) determined by servo calibration.
static constexpr float SERVO_D1_ANG_MIN = 0.0f;
static constexpr float SERVO_D1_ANG_MAX = 0.5f;
static constexpr float SERVO_START_POSITION_D1 = 0.0f; // muss noch kalibriert werden
}

ServoModule::ServoModule() : m_servo(PC_8)
{
}

void ServoModule::initialize()
{
    m_servo.calibratePulseMinMax(SERVO_D1_ANG_MIN, SERVO_D1_ANG_MAX);
    m_servo.setMaxAcceleration(1.0e6f); // no acceleration limit
    if (!m_servo.isEnabled()) {
        m_servo.enable(SERVO_START_POSITION_D1);
    }
}

void ServoModule::setSteeringAngle(float angle)
{
    m_servo.setPulseWidth(angle);
}


void ServoModule::disable()
{
    if (m_servo.isEnabled())
        m_servo.disable();
}
}

namespace arm_drehkranz {
namespace {
// Physical pulse width limits (seconds) determined by servo calibration.
static constexpr float SERVO_D2_ANG_MIN = 0.0f;
static constexpr float SERVO_D2_ANG_MAX = 0.5f;
static constexpr float SERVO_START_POSITION_D2 = 0.0f; // muss noch kalibriert werden
}

ServoModule::ServoModule() : m_servo(PC_6)
{
}

void ServoModule::initialize()
{
    m_servo.calibratePulseMinMax(SERVO_D2_ANG_MIN, SERVO_D2_ANG_MAX);
    m_servo.setMaxAcceleration(1.0e6f); // no acceleration limit
    if (!m_servo.isEnabled()) {
        m_servo.enable(SERVO_START_POSITION_D2);
    }
}

void ServoModule::setSteeringAngle(float angle)
{
    m_servo.setPulseWidth(angle);
}


void ServoModule::disable()
{
    if (m_servo.isEnabled())
        m_servo.disable();
}
}
