#include "color_sensor_module.h"

#include "PESBoardPinMap.h"

static constexpr float CALIB_BLACK_R = 355.97f;
static constexpr float CALIB_BLACK_G = 355.37f;
static constexpr float CALIB_BLACK_B = 1320.07f;
static constexpr float CALIB_BLACK_C = 535.65f;
static constexpr float CALIB_WHITE_R = 2197.95f;
static constexpr float CALIB_WHITE_G = 2336.46f;
static constexpr float CALIB_WHITE_B = 8210.50f;
static constexpr float CALIB_WHITE_C = 3553.85f;

ColorSensorModule::ColorSensorModule() :
    m_sensor(PB_3)
{
    m_sensor.switchLed(OFF);
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
    // Trigger a read path from the sensor thread output.
    (void)m_sensor.readColor();
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
