#include "servo_module_Arm.h"

#include "PESBoardPinMap.h"
// The maximum speed for the servo is determined by testing and calibration.
static constexpr float SERVO_MAX_SPEED = 0.18f;

namespace arm_lenkung {
namespace {
// Physical pulse width limits (seconds) determined by servo calibration.
static constexpr float SERVO_D1_ANG_MIN = 0.035f;
static constexpr float SERVO_D1_ANG_MAX = 0.110f;
static constexpr float SERVO_START_POSITION_D1 = 0.5f;


float constrainNormalized(float value)
{
    return (value > 1.0f) ? 1.0f : (value < 0.0f) ? 0.0f : value;
}

float invertNormalized(float value)
{
    return 1.0f - constrainNormalized(value);
}
}

ServoModule::ServoModule() : m_servo(PC_8)
{
}

void ServoModule::initialize()
{
    m_servo.calibratePulseMinMax(SERVO_D1_ANG_MIN, SERVO_D1_ANG_MAX);
    m_servo.setMaxAcceleration(SERVO_MAX_SPEED);
}

void ServoModule::enable()
{
    if (!m_servo.isEnabled()) {
        m_servo.enable(SERVO_START_POSITION_D1);
    }
}

void ServoModule::setSteeringAngle(float angle)
{
    m_servo.setPulseWidth(invertNormalized(angle));
}

void ServoModule::center()
{
    m_servo.setPulseWidth(SERVO_START_POSITION_D1);
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
static constexpr float SERVO_D2_ANG_MIN = 0.035f;
static constexpr float SERVO_D2_ANG_MAX = 0.110f;
static constexpr float SERVO_START_POSITION_D2 = 0.5f;

float constrainNormalized(float value)
{
    return (value > 1.0f) ? 1.0f : (value < 0.0f) ? 0.0f : value;
}

float invertNormalized(float value)
{
    return 1.0f - constrainNormalized(value);
}
}

ServoModule::ServoModule() : m_servo(PC_6)
{
}

void ServoModule::initialize()
{
    m_servo.calibratePulseMinMax(SERVO_D2_ANG_MIN, SERVO_D2_ANG_MAX);
    m_servo.setMaxAcceleration(SERVO_MAX_SPEED);
}

void ServoModule::enable()
{
    if (!m_servo.isEnabled()) {
        m_servo.enable(SERVO_START_POSITION_D2);
    }
}

void ServoModule::setSteeringAngle(float angle)
{
    m_servo.setPulseWidth(invertNormalized(angle));
}

void ServoModule::center()
{
    m_servo.setPulseWidth(SERVO_START_POSITION_D2);
}


void ServoModule::disable()
{
    if (m_servo.isEnabled())
        m_servo.disable();
}
}
