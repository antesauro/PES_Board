#pragma once

#include <cstdint>

#include "PIDCntrl.h"
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

private:
    SensorBar m_sensorBar;
    PIDCntrl m_pidController;
    float m_steeringCommand;
    float m_driveVoltage;
};
