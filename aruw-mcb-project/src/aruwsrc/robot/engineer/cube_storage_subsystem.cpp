#include "cube_storage_subsystem.hpp"
namespace aruwsrc::robot::engineer
{
    CubeStorageSubsystem::CubeStorageSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& liftMotor)
    : Subsystem(drivers),
      motor(liftMotor){};
void CubeStorageSubsystem::initialize() {
    motor.initialize();
}

void CubeStorageSubsystem::moveMotor(int32_t power) { 
    motor.setDesiredOutput(power); 
}
void CubeStorageSubsystem::refreshSafeDisconnect() {
    motor.setDesiredOutput(0);
}
bool limit = false;

bool CubeStorageSubsystem::isLimitSwitched() { 
    return drivers->digital.read(LIMITSWITCH_PORT); 
}

void CubeStorageSubsystem::refresh() { 
    limit = !drivers->digital.read(LIMITSWITCH_PORT); 
}


}
