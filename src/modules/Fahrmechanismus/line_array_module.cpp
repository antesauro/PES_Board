#include "line_array_module.h"

#include <cmath>
#include <string>

#include "PESBoardPinMap.h"
#include "debug_print.h"

namespace {
static constexpr float STEERING_CENTER = 0.5f;
static constexpr float STEERING_MIN = 0.15f; // min servo position with 1.25 gear ratio
static constexpr float STEERING_MAX = 0.85f; // max servo position with 1.25 gear ratio
static constexpr float DRIVE_VOLTAGE_FULL = 12.0f;

// --- NEW NON-LINEAR CONTROLLER GAINS ---
static constexpr float KP_LINEAR = -0.1f;    // Gentle steering for straightaways
static constexpr float KP_NONLINEAR = -1.6f; // Aggressive factor for sharp curves

static constexpr float CORRECTION_ALPHA = 0.6f;  // Keep the EMA filter for smooth servo action
static constexpr float STEERING_STEP_MAX = 0.3f; // Keep servo fast

static constexpr uint8_t SENSOR_MASK_B2_TO_B5 = 0x3C;
static constexpr uint8_t SENSOR_MASK_ALL_BITS = 0xFF;

float clampf(float value, float minValue, float maxValue)
{
    return (value < minValue) ? minValue : ((value > maxValue) ? maxValue : value);
}
} // namespace

LineArrayModule::LineArrayModule()
    : m_sensorBar(PB_IMU_SDA, PB_IMU_SCL, 0.10f, false)
    , m_steeringCommand(STEERING_CENTER)
    , m_driveVoltage(0.0f)
    , m_filteredCorrection(0.0f)
    , m_centerHoldActive(false)
    , m_centerHoldEnter(0.0f)
    , m_centerHoldExit(0.0f)
{
    m_sensorBar.clearInvertBits();
    m_sensorBar.clearBarStrobe();
}

uint8_t LineArrayModule::update(bool do_print)
{
    m_sensorBar.update();

    const uint8_t raw = m_sensorBar.getRaw();
    const bool lineDetected = m_sensorBar.isAnyLedActive();
    const uint8_t numActiveLeds = m_sensorBar.getNrOfLedsActive(); // gets number of active leds from sensor bar
    float position = 0.0f;

    // Crossing logic
    if (lineDetected) {
        // If the outer LEDs are more than 40% active, it's a horizontal line/crossing!
        if (numActiveLeds >= 4) {
            // Hold current steering angle to go straight across.
            position = m_filteredCorrection;
        } else {
            //  Steer normally.
            position = m_sensorBar.getAngleRad();
        }
    }
    // apply the fast filter
    m_filteredCorrection += CORRECTION_ALPHA * (position - m_filteredCorrection);

    uint8_t event = EVENT_NONE;
    if (raw == SENSOR_MASK_ALL_BITS)
        event = EVENT_DELIVERY_HOUSE;
    else if (raw == SENSOR_MASK_B2_TO_B5)
        event = EVENT_PICKUP_HOUSE;

    // --- NON-LINEAR P CONTROLLER MATH ---
    float err = m_filteredCorrection;
    float steeringOutput = (KP_LINEAR * err) + (KP_NONLINEAR * err * fabsf(err));

    // Apply the output to the target (using subtraction to steer the correct way)
    const float steeringTarget = STEERING_CENTER - steeringOutput;

    const float steeringDelta = clampf(steeringTarget - m_steeringCommand, -STEERING_STEP_MAX, STEERING_STEP_MAX);
    m_steeringCommand = clampf(m_steeringCommand + steeringDelta, STEERING_MIN, STEERING_MAX);

    // --- DRIVE SCALING ---
    const float max_error = 0.5f; // Roughly max angle in radians
    float drive_scale = 1.0f - fabsf(m_filteredCorrection) / max_error;

    // Minimum power in curves
    const float MIN_DRIVE_SCALE = 0.3f;

    if (drive_scale < MIN_DRIVE_SCALE) {
        drive_scale = MIN_DRIVE_SCALE;
    }
    if (drive_scale > 1.0f) {
        drive_scale = 1.0f;
    }

    m_driveVoltage = DRIVE_VOLTAGE_FULL * drive_scale;

    // --- RESET LOGIC ---
    if (!lineDetected) {
        m_filteredCorrection = 0.0f;
        m_steeringCommand = STEERING_CENTER;
        m_driveVoltage = 0.0f;
        m_centerHoldActive = false;
    }

    if (do_print)
        printLineArrayDebug(raw, position, m_filteredCorrection, m_steeringCommand, m_driveVoltage, event);

    return event;
}

float LineArrayModule::steeringCommand() const { return m_steeringCommand; }

float LineArrayModule::driveVoltage() const { return m_driveVoltage; }

void LineArrayModule::setCenterHoldThresholds(float enterThreshold, float exitThreshold)
{
    // Unused, but kept to prevent compilation errors if called from main.cpp
}