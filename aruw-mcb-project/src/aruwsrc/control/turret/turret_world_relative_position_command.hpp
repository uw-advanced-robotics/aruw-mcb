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
#include "aruwlib/control/command.hpp"

#include "aruwsrc/algorithms/turret_pid.hpp"

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
     * @param
     */
    TurretWorldRelativePositionCommand(
        aruwlib::Drivers *drivers,
        TurretSubsystem *subsystem,
        const chassis::ChassisSubsystem *chassis,
        float turretStartAngle,
        float yawKp,
        float yawKi,
        float yawKdTurretImu,
        float yawKdChassisImu,
        float yawMaxICumulative,
        float yawMaxOutput,
        float yawTQDerivativeKalman,
        float yawTRDerivativeKalman,
        float yawTQProportionalKalman,
        float yawTRProportionalKalman,
        float pitchKp,
        float pitchKi,
        float pitchKd,
        float pitchMaxICumulative,
        float pitchMaxOutput,
        float pitchTQDerivativeKalman,
        float pitchTRDerivativeKalman,
        float pitchTQProportionalKalman,
        float pitchTRProportionalKalman,
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

    aruwsrc::algorithms::TurretPid yawPid;
    aruwsrc::algorithms::TurretPid pitchPid;

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
