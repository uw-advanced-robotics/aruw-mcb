#include "sentinel_drive_simulation.hpp"

void SentinelDriveSimulation::setDesiredRpm(float desRpm) 
{
    currentRpm = desRpm;
}

float SentinelDriveSimulation::getRpm()
{
    return currentRpm;
}
    
float SentinelDriveSimulation::absolutePosition() 
{
    return 0.0;
}


