#include "cube_up_command.hpp"
namespace aruwsrc::robot::engineer {
    CubeUpCommand::CubeUpCommand(CubeStorageSubsystem &cubeLift) 
    : cubeLift(cubeLift)
{
    addSubsystemRequirement(&cubeLift);
}

void CubeUpCommand::initialize() {}

void CubeUpCommand::execute() { 
    cubeLift.moveMotor(power); 
}

void CubeUpCommand::end(bool) { 
    cubeLift.moveMotor(0); 
}

bool CubeUpCommand::isFinished() const { 
    return cubeLift.isLimitSwitched(); 
}

}