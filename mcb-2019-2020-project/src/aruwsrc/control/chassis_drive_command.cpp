#include "chassis_drive_command.hpp"

namespace aruwsrc
{

namespace control
{
    ChassisDriveCommand::ChassisDriveCommand(ChassisSubsystem* chassis):
        Command(), chassis(chassis)
    {
        addSubsystemRequirement(reinterpret_cast<Subsystem*>(chassis));
    }

    void ChassisDriveCommand::initialize(){}

    void ChassisDriveCommand::execute(){}

    void ChassisDriveCommand::end(bool interrupted){}

    bool ChassisDriveCommand::isFinished() const {}
}  // namespace control

}  // namespace aruwsrc