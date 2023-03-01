
#ifndef CALIBRATE_SUBSYSTEM_HPP_
#define CALIBRATE_SUBSYSTEM_HPP_

#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "tap/control/subsystem.hpp"

namespace aruwsrc::control::imu
{
class CalibrateSubsystem : public tap::control::Subsystem
{
public:
    CalibrateSubsystem(tap::Drivers* drivers,
        ImuCalibrateCommand& imuCalibrateCommand);

    void calibrateImu();

    const char* getName() override { return "Calibrate"; }
private:
    ImuCalibrateCommand& imuCalibrateCommand;
};
} // namespace aruwsrc::control::imu
#endif