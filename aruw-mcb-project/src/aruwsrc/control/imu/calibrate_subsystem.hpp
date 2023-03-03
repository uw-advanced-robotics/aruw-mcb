
#ifndef CALIBRATE_SUBSYSTEM_HPP_
#define CALIBRATE_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

#include "aruwsrc/control/control_operator_interface.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"

namespace aruwsrc::control::imu
{
class CalibrateSubsystem : public tap::control::Subsystem
{
public:
    CalibrateSubsystem(
        tap::Drivers* drivers,
        ImuCalibrateCommand& imuCalibrateCommand,
        ControlOperatorInterface& controlOperatorInterface);

    void calibrateImu();

    void refresh() override;

    const char* getName() override { return "Calibrate"; }

private:
    ImuCalibrateCommand& imuCalibrateCommand;
    ControlOperatorInterface& controlOperatorInterface;
};
}  // namespace aruwsrc::control::imu
#endif