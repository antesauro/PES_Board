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

    printf("Color sensor=%d (%s) package=%d\n",
           sensor_color_num,
           sensor_color_str,
           package_color_num);
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
