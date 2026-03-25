#include "modules/line_array_module.h"

#include "PESBoardPinMap.h"
#include "debug_print.h"

#include <cmath>

namespace {
static constexpr float GOOD_POSITION          = 0.0f;
static constexpr float MIN_POSITION           = -127.0f;
static constexpr float MAX_POSITION           = 127.0f;
static constexpr float STEERING_CENTER        = 0.5f;
static constexpr float STEERING_MIN           = 0.20f; //min servo position with 1.25  gear ration
static constexpr float STEERING_MAX           = 0.8f;  //max servo position with 1.25  gear ration
static constexpr float DRIVE_VOLTAGE_FULL     = 12.0f;
static constexpr float PID_KP                 = -0.0020f; //steering correction strength (negative to steer in correct direction)
static constexpr float PID_KI                 = 0.0f;
static constexpr float PID_KD                 = 0.0f;
static constexpr float PID_DT_SECONDS         = 0.020f;

static constexpr uint8_t SENSOR_MASK_B2_TO_B5 = 0x3C;
static constexpr uint8_t SENSOR_MASK_ALL_BITS = 0xFF;
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
    m_driveVoltage(0.0f)
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
    const float correction = lineDetected ? (static_cast<float>(position) - GOOD_POSITION) : 0.0f;

    uint8_t event = EVENT_NONE;
    if (raw == SENSOR_MASK_ALL_BITS)
        event = EVENT_DELIVERY_HOUSE;
    else if (raw == SENSOR_MASK_B2_TO_B5)
        event = EVENT_PICKUP_HOUSE;

    m_steeringCommand = STEERING_CENTER - m_pidController.update(correction);

    // Scale drive voltage by how close to centre the line is
    float max_error = fmaxf(fabsf(MAX_POSITION - GOOD_POSITION), fabsf(MIN_POSITION - GOOD_POSITION));
    if (max_error < 1.0e-6f) max_error = 1.0f;
    float drive_scale = 1.0f - fabsf(correction) / max_error;
    if (drive_scale < 0.0f) drive_scale = 0.0f;
    if (drive_scale > 1.0f) drive_scale = 1.0f;
    m_driveVoltage = DRIVE_VOLTAGE_FULL * drive_scale;

    if (!lineDetected) {
        m_pidController.reset();
        m_steeringCommand = STEERING_CENTER;
        m_driveVoltage = 0.0f;
    }

    if (do_print)
        printLineArrayDebug(raw, position, correction, m_steeringCommand, m_driveVoltage, event);

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
