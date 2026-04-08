#pragma once

#include <cstdint>

#include "SensorBar.h"

class LineArrayModule
{
public:
    static constexpr uint8_t EVENT_NONE = 0;
    static constexpr uint8_t EVENT_PICKUP_HOUSE = 1;
    static constexpr uint8_t EVENT_DELIVERY_HOUSE = 2;

    LineArrayModule();

    // Returns event code (EVENT_NONE / EVENT_PICKUP_HOUSE / EVENT_DELIVERY_HOUSE).
    // Pass do_print = true to print raw bits + all values to serial.
    uint8_t update(bool do_print = false);

    float steeringCommand() const;

    // Drive voltage scaled by line deviation: 0 V when line lost, up to DRIVE_VOLTAGE_FULL_POWER.
    float driveVoltage() const;

    // Configure center hold hysteresis thresholds in correction units.
    // enterThreshold should be <= exitThreshold.
    void setCenterHoldThresholds(float enterThreshold, float exitThreshold);

private:
    SensorBar m_sensorBar;
    float m_steeringCommand;
    float m_driveVoltage;
    float m_filteredCorrection;
    bool m_centerHoldActive;
    float m_centerHoldEnter;
    float m_centerHoldExit;
};
