#include "chassis_drive_command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/communication/remote.hpp"

namespace aruwsrc
{

namespace control
{

void ChassisDriveCommand::initialize()
{}

void ChassisDriveCommand::execute()
{
    float remoteMoveX = aruwlib::Remote::getChassisX();
    float remoteMoveY = aruwlib::Remote::getChassisY();

    float chassisMoveX, chassisMoveY, chassisMoveZ;
    
}

void ChassisDriveCommand::end(bool interrupted)
{}

bool ChassisDriveCommand::isFinished() const
{
    return false;
}

}  // namespace control

}  // namespace aruwsrc
