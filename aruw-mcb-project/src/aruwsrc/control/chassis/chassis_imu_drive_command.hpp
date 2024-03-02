/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CHASSIS_IMU_DRIVE_COMMAND_HPP_
#define CHASSIS_IMU_DRIVE_COMMAND_HPP_

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/turret/turret_motor.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"

namespace aruwsrc::chassis
{
class ChassisSubsystem;

/**
 * A command that allows the user to control the translation and rotation of the chassis. Can be
 * used with or without a turret.
 *
 * When no turret is passed into this command upon construction, user translational input is
 * relative to the chassis in a manner similar to `chassis_rel_drive.hpp`. When a turret is passed
 * into this command, user translational input is relative to the turret.
 *
 * The user specifies some rotation via the control operator interface (see
 * the function `getChassisRInput`). The chassis mounted IMU is used as feedback for a position
 * controller so that the user specified chassis rotation is absolute. This means the chassis will
 * attempt to maintain a particular world relative chassis rotation angle.
 *
 * @note It is assumed that the onboard `mpu6500` is attached to the chassis.
 */
class ChassisImuDriveCommand : public tap::control::Command
{
public:
    /**
     * The value to scale the rotation as specified by `controlOperatorInterface.getChassisRInput()`
     * by.
     */
    static constexpr float USER_INPUT_TO_ANGLE_DELTA_SCALAR = 0.000002f;
    /**
     * Maximum error between the actual IMU angle and target angle specified by the user. We cap the
     * rotation error to avoid issues that occur if the robot is picked up, turned around, and
     * placed back. Without limiting the rotation error, the error will be very large and the robot
     * will spin around in an attempt to reach the original setpoint. Instead, the error will only
     * be `MAX_ROTATION_ERR`.
     */
    static constexpr float MAX_ROTATION_ERR = modm::toRadian(30);

    /**
     * Constructs a ChassisImuDriveCommand that will control `chassis`.
     *
     * @param[in] drivers A pointer to the global drivers object.
     * @param[in] chassis A `ChassisSubsystem` that this command will control.
     * @param[in] yawMotor A turret yaw motor that the command will use to drive relative to the
     * turret. If the robot does not have a turret, pass in `nullptr`.
     */
    ChassisImuDriveCommand(
        tap::Drivers* drivers,
        aruwsrc::control::ControlOperatorInterface* operatorInterface,
        ChassisSubsystem* chassis,
        const aruwsrc::control::turret::TurretMotor* yawMotor);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis imu drive"; }

private:
    tap::Drivers* drivers;
    aruwsrc::control::ControlOperatorInterface* operatorInterface;
    ChassisSubsystem* chassis;
    const aruwsrc::control::turret::TurretMotor* yawMotor;
    tap::algorithms::ContiguousFloat rotationSetpoint;
    bool imuSetpointInitialized = false;
    uint32_t prevTime = 0;
};  // class ChassisImuDriveCommand

}  // namespace aruwsrc::chassis

#endif  // CHASSIS_IMU_DRIVE_COMMAND_HPP_
