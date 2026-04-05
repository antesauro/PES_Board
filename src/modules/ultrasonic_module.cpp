#include "ultrasonic_module.h"

#include "PESBoardPinMap.h"

UltrasonicModule::UltrasonicModule() :
    m_sensor(PB_D3),
    m_distanceCm(0.0f),
    m_distanceMinCm(6.0f),
    m_distanceMaxCm(40.0f)
{
}

void UltrasonicModule::update()
{
    const float distanceCandidate = m_sensor.read();
    if (distanceCandidate > m_distanceMinCm && distanceCandidate < m_distanceMaxCm)
        m_distanceCm = distanceCandidate;
}

void UltrasonicModule::reset()
{
    m_distanceCm = 0.0f;
}

float UltrasonicModule::distanceCm() const
{
    return m_distanceCm;
}
