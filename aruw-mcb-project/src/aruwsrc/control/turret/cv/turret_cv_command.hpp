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

#ifndef TURRET_CV_COMMAND_HPP_
#define TURRET_CV_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "../algorithms/turret_controller_interface.hpp"
#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"

namespace tap::control::odometry
{
class Odometry2DInterface;
}

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::turret
{
class TurretSubsystem;
}

namespace aruwsrc::control::launcher
{
class RefereeFeedbackFrictionWheelSubsystem;
}

namespace aruwsrc::chassis
{
class ChassisSubsystem;
}

namespace aruwsrc::control::turret::cv
{
/**
 * A command that receives input from the vision system via the `VisionCoprocessor` driver and
 * aims the turret accordingly using a position PID controller.
 *
 * This command, unlike the `SentinelTurretCVCommand`, is not responsible for firing projectiles
 * when the auto aim system determines it should fire. Nor does this class scan the turret back and
 * forth.
 *
 * @note If the auto aim system is offline, does not have a target acquired, or has an invalid
 * target (for example, the target is too far away), then user input from the
 * `ControlOperatorInterface` is used to control the turret instead.
 */
class TurretCVCommand : public tap::control::Command
{
public:
    /**
     * Constructs a TurretCVCommand
     *
     * @param[in] drivers Pointer to a global drivers object.
     * @param[in] turretSubsystem Pointer to the turret to control.
     * @param[in] yawController Pointer to a yaw controller that will be used to control the yaw
     * axis of the turret.
     * @param[in] pitchController Pointer to a pitch controller that will be used to control the
     * pitch axis of the turret.
     * @param[in] odometryInterface Odometry object, used for position odometry information.
     * @param[in] frictionWheels Friction wheels, used to determine the launch speed because leading
     * a target is a function of how fast a projectile is launched at.
     * @param[in] userPitchInputScalar When user input is used, this scalar is used to scale the
     * pitch user input.
     * @param[in] userYawInputScalar When user input is used, this scalar is used to scale the yaw
     * user input.
     * @param[in] defaultLaunchSpeed The launch speed to be used in ballistics computation when the
     * friction wheels report the launch speed is 0 (i.e. when the friction wheels are off).
     * @param[in] turretID
     */
    TurretCVCommand(
        aruwsrc::Drivers *drivers,
        TurretSubsystem *turretSubsystem,
        algorithms::TurretYawControllerInterface *yawController,
        algorithms::TurretPitchControllerInterface *pitchController,
        const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
        const control::launcher::RefereeFeedbackFrictionWheelSubsystem &frictionWheels,
        const float userPitchInputScalar,
        const float userYawInputScalar,
        const float defaultLaunchSpeed,
        uint8_t turretID = 0);

    void initialize() override;

    bool isReady() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override;

    const char *getName() const override { return "turret CV"; }

private:
    aruwsrc::Drivers *drivers;

    uint8_t turretID;

    TurretSubsystem *turretSubsystem;

    algorithms::TurretYawControllerInterface *yawController;
    algorithms::TurretPitchControllerInterface *pitchController;

    aruwsrc::algorithms::OttoBallisticsSolver ballisticsSolver;

    const float userPitchInputScalar;
    const float userYawInputScalar;

    uint32_t prevTime;
    void computeTurretAimAngles(float *pitch, float *yaw);
};
}  // namespace aruwsrc::control::turret::cv

#endif  // TURRET_CV_COMMAND_HPP_
