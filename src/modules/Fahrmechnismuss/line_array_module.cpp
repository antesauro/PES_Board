#include "line_array_module.h"

#include "PESBoardPinMap.h"
#include "debug_print.h"

#include <cmath>

namespace {
static constexpr float STEERING_CENTER        = 0.5f;
static constexpr float STEERING_MIN           = 0.20f; //min servo position with 1.25  gear ration
static constexpr float STEERING_MAX           = 0.8f;  //max servo position with 1.25  gear ration
static constexpr float DRIVE_VOLTAGE_FULL     = 12.0f;
static constexpr float PID_KP                 = -0.7f; //steering correction strength (negative to steer in correct direction)
static constexpr float PID_KI                 = 0.0f;
static constexpr float PID_KD                 = -1.5f; //steering damping (negative to steer in correct direction)
static constexpr float PID_DT_SECONDS         = 0.02f;// PID update interval in seconds, should match main loop period for best performance
static constexpr float CORRECTION_DEADBAND    = 1.0f;
static constexpr float CORRECTION_ALPHA       = 0.35f;
static constexpr float STEERING_STEP_MAX      = 0.015f;
static constexpr float CENTER_HOLD_ENTER      = 2.0f;
static constexpr float CENTER_HOLD_EXIT       = 4.0f;

static constexpr uint8_t SENSOR_MASK_B2_TO_B5 = 0x3C;
static constexpr uint8_t SENSOR_MASK_ALL_BITS = 0xFF;

float clampf(float value, float minValue, float maxValue)
{
    return (value < minValue) ? minValue : ((value > maxValue) ? maxValue : value);
}
}

LineArrayModule::LineArrayModule() :
    m_sensorBar(PB_IMU_SDA, PB_IMU_SCL, 0.10f, false),

    m_pidController(PID_KP,
                    PID_KI,
                    PID_KD,
                    PID_DT_SECONDS,
                    STEERING_MIN - STEERING_CENTER,
                    STEERING_MAX - STEERING_CENTER),
    m_steeringCommand(STEERING_CENTER),
    m_driveVoltage(0.0f),
    m_filteredCorrection(0.0f),
    m_centerHoldActive(false)
{
    m_sensorBar.clearInvertBits();
    m_sensorBar.clearBarStrobe();
}

uint8_t LineArrayModule::update(bool do_print)
{
    m_sensorBar.update();

    const uint8_t raw = m_sensorBar.getRaw();
    const bool lineDetected = m_sensorBar.isAnyLedActive();
    const int8_t position = lineDetected ? m_sensorBar.getBinaryPosition() : 0;
    const float rawCorrection = lineDetected ? (static_cast<float>(position) ) : 0.0f;

    float correction = rawCorrection;
    const float absCorrection = fabsf(correction);

    // Hold steering at center for small sensor toggles around the line center.
    if (m_centerHoldActive) {
        if (absCorrection < CENTER_HOLD_EXIT) {
            correction = 0.0f;
        } else {
            m_centerHoldActive = false;
        }
    }

    if (!m_centerHoldActive && absCorrection < CENTER_HOLD_ENTER) {
        m_centerHoldActive = true;
        correction = 0.0f;
    }

    if (fabsf(correction) < CORRECTION_DEADBAND)
        correction = 0.0f;

    m_filteredCorrection += CORRECTION_ALPHA * (correction - m_filteredCorrection);

    uint8_t event = EVENT_NONE;
    if (raw == SENSOR_MASK_ALL_BITS)
        event = EVENT_DELIVERY_HOUSE;
    else if (raw == SENSOR_MASK_B2_TO_B5)
        event = EVENT_PICKUP_HOUSE;

    const float steeringTarget = STEERING_CENTER - m_pidController.update(m_filteredCorrection);
    const float steeringDelta = clampf(steeringTarget - m_steeringCommand, -STEERING_STEP_MAX, STEERING_STEP_MAX);
    m_steeringCommand = clampf(m_steeringCommand + steeringDelta, STEERING_MIN, STEERING_MAX);

    // Scale drive voltage by how close to centre the line is
    const float max_error = 127.0f;
    float drive_scale = 1.1f - fabsf(m_filteredCorrection) / max_error;
    if (drive_scale < 0.0f) drive_scale = 0.0f;
    if (drive_scale > 1.0f) drive_scale = 1.0f;
    m_driveVoltage = DRIVE_VOLTAGE_FULL * drive_scale;

    if (!lineDetected) {
        m_pidController.reset();
        m_filteredCorrection = 0.0f;
        m_steeringCommand = STEERING_CENTER;
        m_driveVoltage = 0.0f;
        m_centerHoldActive = false;
    }

    if (do_print)
        printLineArrayDebug(raw, position, m_filteredCorrection, m_steeringCommand, m_driveVoltage, event);

    return event;
}

float LineArrayModule::steeringCommand() const
{
    return m_steeringCommand;
}

float LineArrayModule::driveVoltage() const
{
    return m_driveVoltage;
}
