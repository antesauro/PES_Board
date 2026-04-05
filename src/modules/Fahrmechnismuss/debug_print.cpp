#include "debug_print.h"

#include <cstdio>

#include "color_sensor_module.h"

void printInitialState()
{
#if DEBUG_PRINT_ENABLE
    // printf("INITIAL\n");
#endif
}

void printReadyState()
{
#if DEBUG_PRINT_ENABLE
    // printf("READY\n");
#endif
}

void printRetrieveState()
{
#if DEBUG_PRINT_ENABLE
    // printf("RETRIEVE\n");
#endif
}

void printPickupState()
{
#if DEBUG_PRINT_ENABLE
    // printf("PICKUP\n");
#endif
}

void printDeliverState()
{
#if DEBUG_PRINT_ENABLE
    // printf("DELIVER\n");
#endif
}

void printSleepState()
{
#if DEBUG_PRINT_ENABLE
    // printf("SLEEP\n");
#endif
}

void printEmergencyState()
{
#if DEBUG_PRINT_ENABLE
    // printf("EMERGENCY\n");
#endif
}

void printDriveStatus(ColorSensorModule& color_sensor_module)
{
#if DEBUG_PRINT_ENABLE
    color_sensor_module.printColor();
    // color_sensor_module.printAverage();
#else
    (void)color_sensor_module;
#endif
}

void printPickupHouseDistanceMm(int distance_mm)
{
#if DEBUG_PRINT_ENABLE
    // printf("Querlinie Abhol-Haus: %d mm\n", distance_mm);
#else
    (void)distance_mm;
#endif
}

void printDeliveryHouseDistanceMm(int distance_mm)
{
#if DEBUG_PRINT_ENABLE
    // printf("Querlinie Abliefer-Haus: %d mm\n", distance_mm);
#else
    (void)distance_mm;
#endif
}

void printMainLoopOverrunWarning()
{
#if DEBUG_PRINT_ENABLE
    // printf("Warning: Main task took longer than main_task_period_ms\n");
#endif
}

void printLineArrayDebug(uint8_t raw,
                         int8_t position,
                         float correction,
                         float steering_command,
                         float drive_voltage,
                         uint8_t event_code)
{
#if DEBUG_PRINT_ENABLE
     for (int bit = 7; bit >= 0; bit--)
        printf("%d", (raw >> bit) & 0x01);

     printf(" pos=%d corr=%.2f servo=%.3f motor=%.2f event=%d\n",
        position,
        correction,
        steering_command,
        drive_voltage,
        event_code);
#else
    (void)raw;
    (void)position;
    (void)correction;
    (void)steering_command;
    (void)drive_voltage;
    (void)event_code;
#endif
}

void printColorAverageHz(const float avg_hz[4])
{
#if DEBUG_PRINT_ENABLE
    printf("Color Avg Hz: %f %f %f %f\n", avg_hz[0], avg_hz[1], avg_hz[2], avg_hz[3]);
#else
    (void)avg_hz;
#endif
}
