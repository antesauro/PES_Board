#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H

#include <stdint.h>

class ColorSensorModule;

// Central debug-print switch.
// 0 = all output off, 1 = output on.
#define DEBUG_PRINT_ENABLE 1

void printInitialState();
void printReadyState();
void printRetrieveState();
void printPickupState();
void printDeliverState();
void printDepartState();
void printSleepState();
void printEmergencyState();

void printDriveStatus(ColorSensorModule& color_sensor_module);

void printPickupHouseDistanceMm(int distance_mm);
void printDeliveryHouseDistanceMm(int distance_mm);
void printMainLoopOverrunWarning();

void printLineArrayDebug(uint8_t raw,
                         int8_t position,
                         float correction,
                         float steering_command,
                         float drive_voltage,
                         uint8_t event_code);

void printColorAverageHz(const float avg_hz[4]);

#endif
