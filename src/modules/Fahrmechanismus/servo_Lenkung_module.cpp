#include "servo_module.h"

#include "PESBoardPinMap.h"

namespace {
// Physical pulse width limits (seconds) determined by servo calibration.
static constexpr float SERVO_D0_ANG_MIN = 0.035f;
static constexpr float SERVO_D0_ANG_MAX = 0.110f;
// Center of the servo travel range (normalized 0..1 position, NOT a duty cycle).
static constexpr float SERVO_CENTER_ANGLE = 0.5f;
}

ServoModule::ServoModule() : m_servoD0(PB_D0)
{
}

void ServoModule::initialize()
{
    m_servoD0.calibratePulseMinMax(SERVO_D0_ANG_MIN, SERVO_D0_ANG_MAX);
    m_servoD0.setMaxAcceleration(1.0e6f);// no acceleration limit
    if (!m_servoD0.isEnabled()) {
        // Enable directly at center to avoid an initial jump to 0.0.
        m_servoD0.enable(SERVO_CENTER_ANGLE);
    }
}

void ServoModule::setSteeringAngle(float angle)
{
    // angle: 0.0..1.0 normalized position -> Servo lib maps this to
    // SERVO_D0_ANG_MIN..SERVO_D0_ANG_MAX pulse widths internally.
    m_servoD0.setPulseWidth(angle);
}

void ServoModule::center()
{
    m_servoD0.setPulseWidth(SERVO_CENTER_ANGLE);
}

void ServoModule::disable()
{
    if (m_servoD0.isEnabled())
        m_servoD0.disable();
}
