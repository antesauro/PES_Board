#pragma once

#include "ColorSensor.h"

class ColorSensorModule
{
public:
    ColorSensorModule();

    void update();
    void printAverage() const;

private:
    ColorSensor m_sensor;
    float m_avgHz[4];
};
