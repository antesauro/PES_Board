#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H

#include <stdint.h>

class ColorSensorModule;

// Zentrale Schaltstelle für Debug-Ausgaben.
// 0 = alle Ausgaben aus, 1 = Ausgaben an.
#define DEBUG_PRINT_ENABLE 1

void printInitialState();
void printReadyState();
void printRetrieveState();
void printPickupState();
void printDeliverState();
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
