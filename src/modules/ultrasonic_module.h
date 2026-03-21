#pragma once

#include "UltrasonicSensor.h"

class UltrasonicModule
{
public:
    UltrasonicModule();

    void update();
    void reset();
    float distanceCm() const;

private:
    UltrasonicSensor m_sensor;
    float m_distanceCm;
    float m_distanceMinCm;
    float m_distanceMaxCm;
};
