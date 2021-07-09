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

#ifndef TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP_
#define TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP_

#include "aruwlib/algorithms/contiguous_float.hpp"
#include "aruwlib/algorithms/smooth_pid.hpp"
#include "aruwlib/control/command.hpp"

namespace aruwlib
{
class Drivers;
}

namespace aruwsrc
{
namespace chassis
{
class ChassisSubsystem;
}

namespace control::turret
{
class TurretSubsystem;

/**
 * Turret control, with the yaw gimbal using the world relative frame, such that the
 * desired turret angle is independent of the direction that the chassis is facing
 * or rotating. Assumes the Mpu6500 used for calculations is mounted on the chassis.
 */
class TurretWorldRelativePositionCommand : public aruwlib::control::Command
{
public:
    /**
     * This command requires the turret subsystem from a command/subsystem framework perspective.
     * The `ChassisSubsystem` is only used for for odometry information.
     *
     * @param[in] drivers Pointer to a drivers singleton object.
     * @param[in] subsystem The turret subsystem to control.
     * @param[in] chassis The chassis subsystem to reference while running the turret controller.
     * @param[in] turretStartAngle The starting target angle for the pitch/yaw motors.
     * @param[in] yawKdTurretImu The derivative gain to be used while the IMU is on the turret.
     * @param[in] yawKdChassisImu The derivative gain to be used while the IMU is on the chassis.
     * @param[in] yawPidConfig PID configuration for yaw controller.
     * @param[in] pitchPidConfig PID configuration for pitch controller.
     * @param[in] userYawInputScalar Scaler to multiply user input by.
     * @param[in] userPitchInputScalar @see userYawInputScalar.
     * @param[in] pitchGravityCompensationKp Gravity compensation proportional gain.
     * @param[in] useImuOnTurret `true` if the IMU is on the turret. The `imuRxHandler` is used
     *      if this is the case, otherwise if `false`, uses the chassis IMU (the IMU on the MCB).
     */
    TurretWorldRelativePositionCommand(
        aruwlib::Drivers *drivers,
        TurretSubsystem *subsystem,
        const chassis::ChassisSubsystem *chassis,
        float turretStartAngle,
        float yawKdTurretImu,
        float yawKdChassisImu,
        const aruwlib::algorithms::PidConfigStruct &yawPidConfig,
        const aruwlib::algorithms::PidConfigStruct &pitchPidConfig,
        float userYawInputScalar,
        float userPitchInputScalar,
        float pitchGravityCompensationKp,
        bool useImuOnTurret = false);

    void initialize() override;

    bool isFinished() const override { return false; }

    void execute() override;

    void end(bool) override;

    const char *getName() const override { return "turret world relative position"; }

private:
    const float USER_YAW_INPUT_SCALAR;
    const float USER_PITCH_INPUT_SCALAR;
    const float PITCH_GRAVITY_COMPENSATION_KP;
    const float YAW_D_TURRET_IMU;
    const float YAW_D_CHASSIS_IMU;

    aruwlib::Drivers *drivers;

    TurretSubsystem *turretSubsystem;
    const chassis::ChassisSubsystem *chassisSubsystem;

    aruwlib::algorithms::ContiguousFloat yawTargetAngle;

    aruwlib::algorithms::ContiguousFloat currValueImuYawGimbal;

    float imuInitialYaw;

    uint32_t prevTime;

    aruwlib::algorithms::SmoothPid yawPid;
    aruwlib::algorithms::SmoothPid pitchPid;

    const bool useImuOnTurret;
    bool usingImuOnTurret;

    int blinkCounter = 0;

    void runYawPositionController(float dt);
    void runPitchPositionController(float dt);

    float projectChassisRelativeYawToWorldRelative(float yawAngle, float imuInitialAngle);
    float projectWorldRelativeYawToChassisFrame(float yawAngle, float imuInitialAngle);
};  // class TurretWorldRelativePositionCommand

}  // namespace control::turret

}  // namespace aruwsrc

#endif  // TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP_
