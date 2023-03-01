#ifndef WHEEL_IMU_CALIBRATE_COMMAND_HPP_
#define WHEEL_IMU_CALIBRATE_COMMAND_HPP_

#include "aruwsrc/control/imu/calibrate_subsystem.hpp"
#include "tap/drivers.hpp"
#include "aruwsrc/control/control_operator_interface.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "tap/control/command.hpp"

namespace aruwsrc::control::imu {
    
class WheelImuCalibrateCommand : public tap::control::Command
{
    public:
        WheelImuCalibrateCommand(
            CalibrateSubsystem& calibrator,
            tap::Drivers& drivers,
            aruwsrc::control::ControlOperatorInterface& controlOperatorInterface
        );

        void initialize() override;

        void execute() override;

        bool isFinished() const override { return false; }

        void end(bool interrupt) override;

        const char* getName() const override { return "Wheel IMU calibrate command"; }
    private:
        CalibrateSubsystem& calibrator;
        tap::Drivers& drivers;
        aruwsrc::control::ControlOperatorInterface& controlOperatorInterface;
};
    
} // namespace aruwsrc::control::imu

#endif