
#include "calibrate_subsystem.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::control::imu
{

CalibrateSubsystem::CalibrateSubsystem(tap::Drivers* drivers,
        ImuCalibrateCommand& imuCalibrateCommand) : 
        Subsystem(drivers),
        imuCalibrateCommand(imuCalibrateCommand) 
        {}

void CalibrateSubsystem::calibrateImu() {
    if (!drivers->commandScheduler.isCommandScheduled(&imuCalibrateCommand)) {
        
        drivers->commandScheduler.addCommand(&imuCalibrateCommand);
    }
}

} // namespace aruwsrc::control::imu