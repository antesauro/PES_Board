#include "color_sensor_module.h"

#include "PESBoardPinMap.h"

static constexpr float CALIB_BLACK_R = 512.19f;
static constexpr float CALIB_BLACK_G = 514.56f;
static constexpr float CALIB_BLACK_B = 795.54f;
static constexpr float CALIB_BLACK_C = 1963.94f;
static constexpr float CALIB_WHITE_R = 2582.65f;
static constexpr float CALIB_WHITE_G = 2831.28f;
static constexpr float CALIB_WHITE_B = 4452.38f;
static constexpr float CALIB_WHITE_C = 10288.24f;

static int sensorColorToPackageColor(int sensor_color)
{
    switch (sensor_color) {
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

    const float* raw  = m_sensor.readColor();      // filtered Hz [R,G,B,C]
    const float* cal  = m_sensor.readColorCalib(); // white-balanced values the classifier uses
    const float* norm = m_sensor.readColorNorm();  // normalized values

    printf("sensor=%d (%s) | Hz R:%.1f G:%.1f B:%.1f C:%.1f | cal R:%.3f G:%.3f B:%.3f | norm R:%.3f G:%.3f B:%.3f\n",
           sensor_color_num, sensor_color_str,
           raw[0],  raw[1],  raw[2],  raw[3],
           cal[0],  cal[1],  cal[2],
           norm[0], norm[1], norm[2]);
}

int ColorSensorModule::detectedPackageColor()
{
    const uint32_t now_ms = Kernel::Clock::now().time_since_epoch().count() / 1000;
    const int package_color = sensorColorToPackageColor(m_sensor.getColor());

    if (package_color != 0) {
        m_last_package_color = package_color;
        m_last_package_color_ms = now_ms;
        return package_color;
    }

    if (m_last_package_color != 0 && (now_ms - m_last_package_color_ms) <= PACKAGE_COLOR_HOLD_MS) {
        return m_last_package_color;
    }

    return 0;
}

void ColorSensorModule::resetPackageColorHold()
{
    m_last_package_color = 0;
    m_last_package_color_ms = 0;
}
