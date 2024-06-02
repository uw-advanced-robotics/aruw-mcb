/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef SENTRY_IMU_CALIBRATE_COMMAND_HPP_
#define SENTRY_IMU_CALIBRATE_COMMAND_HPP_

#include <vector>

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/communication/mcb-lite/mcb_lite.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_chassis_world_yaw_observer.hpp"
#include "aruwsrc/robot/sentry/sentry_kf_odometry_2d_subsystem.hpp"
namespace aruwsrc::control::imu
{
/**
 * A command whose job is to perform a calibration of the turret and chassis IMUs. Requires that the
 * robot has a turret and a chassis subsystem. Also requires that a turret IMU is connected via the
 * TurretMCBCanComm object.
 *
 * When this command is scheduled, it performs the following actions:
 * 1. Wait until the turret is online and either the chassis mpu6500 or turret MCB IMU is online.
 * 2. Command the pitch and yaw turret gimbals to move to PI/2 radians (forward and flat).
 * 3. Command the chassis to stay still.
 * 4. Pause until the chassis/turret subsystems are no longer moving.
 * 5. Send a calibration signal to the turret MCB.
 * 6. Send signal to onboard IMU to recalibrate.
 * 7. Wait until calibration is complete and then end the command.
 */
class SentryImuCalibrateCommand : public imu::ImuCalibrateCommand
{
public:
    /**
     * Threshold around 0 where turret pitch and yaw velocity is considered to be 0, in radians/s
     */
    static constexpr float VELOCITY_ZERO_THRESHOLD = modm::toRadian(1e-2);
    /**
     * Threshold around 0 where turret pitch and yaw position from the center considered to be 0,
     * in radians
     */
    static constexpr float POSITION_ZERO_THRESHOLD = modm::toRadian(0.2f);

    /**
     * @param[in] drivers A pointer to the global drivers object.
     * @param[in] turretsAndControllers A list of TurretIMUCalibrationConfig structs containing
     * turret and turret IMU information necessary for calibrating the IMU
     * @param[in] chassis A `ChassisSubsystem` that this command will control (will set the desired
     * movement to 0).
     */

    SentryImuCalibrateCommand(
        tap::Drivers *drivers,
        const std::vector<TurretIMUCalibrationConfig> &turretsAndControllers,
        aruwsrc::control::turret::YawTurretSubsystem &turretMajor,
        aruwsrc::control::turret::algorithms::TurretYawControllerInterface &turretMajorController,
        chassis::HolonomicChassisSubsystem &chassis,
        aruwsrc::sentry::SentryChassisWorldYawObserver &yawObserver,
        aruwsrc::sentry::SentryKFOdometry2DSubsystem &odometryInterface,
        aruwsrc::virtualMCB::MCBLite &majorMCBLite,
        aruwsrc::virtualMCB::MCBLite &chassisMCBLite);

    const char *getName() const override { return "Sentry calibrate IMU"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

protected:
    aruwsrc::control::turret::YawTurretSubsystem &turretMajor;
    aruwsrc::control::turret::algorithms::TurretYawControllerInterface &turretMajorController;

    aruwsrc::sentry::SentryChassisWorldYawObserver &yawObserver;

    aruwsrc::sentry::SentryKFOdometry2DSubsystem &odometryInterface;
    aruwsrc::virtualMCB::MCBLite &majorMCBLite;
    aruwsrc::virtualMCB::MCBLite &chassisMCBLite;

    // const std::vector<aruwsrc::virtualMCB::MCBLite *> &mcbLite;
};
}  // namespace aruwsrc::control::imu

#endif  // SENTRY_IMU_CALIBRATE_COMMAND_HPP_
