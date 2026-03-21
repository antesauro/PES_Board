#include "modules/color_sensor_module.h"

#include "PESBoardPinMap.h"
#include "debug_print.h"
#include "mbed.h"

ColorSensorModule::ColorSensorModule() :
    m_sensor(PB_3),
    m_avgHz{0.0f, 0.0f, 0.0f, 0.0f}
{
    m_sensor.switchLed(ON);
}

void ColorSensorModule::update()
{
    const float* avg_hz = m_sensor.readColor();
    for (int i = 0; i < 4; i++)
        m_avgHz[i] = avg_hz[i];
}

void ColorSensorModule::printAverage() const
{
    printColorAverageHz(m_avgHz);
}
