#pragma once

#include "ColorSensor.h"

class ColorSensorModule
{
public:
    ColorSensorModule();

    void calibrate();
    void update();
    void printColor();
    void printAverage() const;

    int detectedPackageColor();

private:
    ColorSensor m_sensor;
    float m_avgHz[4];
};
