#include "cube_down_command.hpp"
namespace aruwsrc::robot::engineer {
    CubeDownCommand::CubeDownCommand(CubeStorageSubsystem &cubeLift) 
    : cubeLift(cubeLift)
{
    addSubsystemRequirement(&cubeLift);
}

void CubeDownCommand::initialize() {}

void CubeDownCommand::execute() { 
    cubeLift.moveMotor(power); 
}

void CubeDownCommand::end(bool) { 
    cubeLift.moveMotor(0); 
}

bool CubeDownCommand::isFinished() const { 
    return cubeLift.isLimitSwitched(); 
}

}