#include "cube_move_command.hpp"
namespace aruwsrc::robot::engineer {
    CubeMoveCommand::CubeMoveCommand(CubeStorageSubsystem &cubeLift) 
    : cubeLift(cubeLift)
{
    addSubsystemRequirement(&cubeLift);
}

void CubeMoveCommand::initialize() {}

void CubeMoveCommand::execute() { 
    cubeLift.moveMotor(power); 
}

void CubeMoveCommand::end(bool) { 
    cubeLift.moveMotor(0); 
}

bool CubeMoveCommand::isFinished() const { 
    return cubeLift.isLimitSwitched(); 
}

}