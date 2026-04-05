#include "color_sensor_module.h"

#include "PESBoardPinMap.h"
#include "debug_print.h"
#include "mbed.h"

// Calibration values measured at 4 cm distance with sensor LED on
// Black reference: Color Avg Hz: 2790 2823 4405 11136
// White reference: Color Avg Hz: 5767 4938 7711 14064
static constexpr float CALIB_BLACK_R = 2790.0f;
static constexpr float CALIB_BLACK_G = 2823.0f;
static constexpr float CALIB_BLACK_B = 4405.0f;
static constexpr float CALIB_BLACK_C = 11136.0f;
static constexpr float CALIB_WHITE_R = 5767.0f;
static constexpr float CALIB_WHITE_G = 4938.0f;
static constexpr float CALIB_WHITE_B = 7711.0f;
static constexpr float CALIB_WHITE_C = 14064.0f;

ColorSensorModule::ColorSensorModule() :
    m_sensor(PB_3),
    m_avgHz{0.0f, 0.0f, 0.0f, 0.0f}
{
    m_sensor.switchLed(ON);
    calibrate();
}

void ColorSensorModule::calibrate()
{
    m_sensor.setCustomCalibration(
        CALIB_BLACK_R, CALIB_BLACK_G, CALIB_BLACK_B, CALIB_BLACK_C,
        CALIB_WHITE_R, CALIB_WHITE_G, CALIB_WHITE_B, CALIB_WHITE_C
    );
}

void ColorSensorModule::update()
{
    const float* avg_hz = m_sensor.readColor();
    for (int i = 0; i < 4; i++)
        m_avgHz[i] = avg_hz[i];
}

void ColorSensorModule::printColor()
{
    const int sensor_color_num = m_sensor.getColor();
    const char* sensor_color_str = m_sensor.getColorString(sensor_color_num);
    const int package_color_num = detectedPackageColor();

    printf("Color sensor=%d (%s) package=%d\n",
           sensor_color_num,
           sensor_color_str,
           package_color_num);
}

void ColorSensorModule::printAverage() const
{
    printColorAverageHz(m_avgHz);
}

int ColorSensorModule::detectedPackageColor()
{
    const int color_num = m_sensor.getColor();

    switch (color_num) {
        case 3:
            return 1; // RED
        case 7:
            return 2; // BLUE
        case 4:
            return 3; // YELLOW
        case 5:
            return 4; // GREEN
        default:
            return 0; // UNKNOWN / not one of the 4 target colors
    }
}
