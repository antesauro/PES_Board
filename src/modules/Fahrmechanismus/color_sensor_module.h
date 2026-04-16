#pragma once

#include "ColorSensor.h"

class ColorSensorModule
{
public:
    ColorSensorModule();

    void update();
    void printColor();

    int detectedPackageColor();

private:
    void calibrate();

    ColorSensor m_sensor;
};
