

#include "wheel_imu_calibrate_command.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::control::imu {
    
    WheelImuCalibrateCommand::WheelImuCalibrateCommand(
        aruwsrc::control::imu::CalibrateSubsystem& calibrator,
        tap::Drivers& drivers,
        aruwsrc::control::ControlOperatorInterface& controlOperatorInterface) 
        : calibrator(calibrator),
        drivers(drivers),
        controlOperatorInterface(controlOperatorInterface)
        {
            addSubsystemRequirement(&calibrator);
        }

    void WheelImuCalibrateCommand::initialize() {}

    void WheelImuCalibrateCommand::execute()
    {
        if (controlOperatorInterface.getCalibrationWheelInput()) {
            calibrator.calibrateImu();
        }
    }

    void WheelImuCalibrateCommand::end(bool) 
    {
        //TODO
    }
    
} // namespace aruw::control::imu