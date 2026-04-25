#pragma once

#include "ColorSensor.h"

class ColorSensorModule
{
public:
    ColorSensorModule();

    void update();
    void printColor();

    int detectedPackageColor();
    void resetPackageColorHold();

private:
    void calibrate();

    static constexpr uint32_t PACKAGE_COLOR_HOLD_MS = 1200;

    ColorSensor m_sensor;
    int m_last_package_color = 0;
    uint32_t m_last_package_color_ms = 0;
};
