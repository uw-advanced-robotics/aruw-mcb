#include "auto_nav_comand.hpp"


namespace aruwsrc::control
{

AutoNavCommand::AutoNavCommand(
    tap::Drivers& drivers,
    HolonomicChassisSubsystem& chassis,
    aruwsrc::control::ControlOperatorInterface& operatorInterface)
    : drivers(drivers),
      chassis(chassis),
      operatorInterface(operatorInterface)
{
    
}

AutoNavCommand::initialize()
{

}

AutoNavCommand::execute()
{
    // TODO: make always beyblading
    // TODO: ramp
    AutoNavSetpointData& setpointData = drivers.visionCoprocessor.getLastSetpointData();
    chassis.setDesiredOutput(setpointData.x, setpointData.y, 0);
}

AutoNavCommand::end(bool)
{
    
}

}  // namespace aruwsrc::control