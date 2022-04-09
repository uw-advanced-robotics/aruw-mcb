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

#ifndef SENTINEL_TURRET_CV_COMMAND_HPP_
#define SENTINEL_TURRET_CV_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "../algorithms/turret_controller_interface.hpp"
#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/control/turret/cv/sentinel_turret_cv_command.hpp"

#include "setpoint_scanner.hpp"

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
class SentinelTurretCVCommand : public tap::control::Command
{
public:
    /**
     * Command will shoot when turret pitch and yaw are both respectively within `FIRING_TOLERANCE`
     * degrees of the ballistics solution.
     */
    static constexpr float FIRING_TOLERANCE = 0.5f;

    /**
     * Constructs a TurretCVCommand
     *
     * @param[in] drivers Pointer to a global drivers object.
     * @param[in] turretSubsystem Pointer to the turret to control.
     * @param[in] yawController Pointer to a yaw controller that will be used to control the yaw
     * axis of the turret.
     * @param[in] pitchController Pointer to a pitch controller that will be used to control the
     * pitch axis of the turret.
     * @param[in] firingCommand Pointer to command to schedule when this command deems it's time to
     * shoot.
     * @param[in] odometryInterface Odometry object, used for position odometry information.
     * @param[in] frictionWheels Friction wheels, used to determine the launch speed because leading
     * a target is a function of how fast a projectile is launched at.
     * @param[in] userPitchInputScalar When user input is used, this scalar is used to scale the
     * pitch user input.
     * @param[in] userYawInputScalar When user input is used, this scalar is used to scale the yaw
     * user input.
     * @param[in] defaultLaunchSpeed The launch speed to be used in ballistics computation when the
     * friction wheels report the launch speed is 0 (i.e. when the friction wheels are off).
     * @param[in] turretID The vision turet ID, must be a valid 0-based index, see VisionCoprocessor
     * for more information.
     */
    SentinelTurretCVCommand(
        aruwsrc::Drivers *drivers,
        TurretSubsystem *turretSubsystem,
        algorithms::TurretYawControllerInterface *yawController,
        algorithms::TurretPitchControllerInterface *pitchController,
        Command *const firingCommand,
        const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
        const control::launcher::RefereeFeedbackFrictionWheelSubsystem &frictionWheels,
        const float userPitchInputScalar,
        const float userYawInputScalar,
        const float defaultLaunchSpeed,
        const uint8_t turretID);

    void initialize() override;

    bool isReady() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override;

    const char *getName() const override { return "turret CV"; }

private:
    aruwsrc::Drivers *drivers;

    TurretSubsystem *turretSubsystem;

    algorithms::TurretYawControllerInterface *yawController;
    algorithms::TurretPitchControllerInterface *pitchController;

    const uint8_t turretID;

    /**
     * The command to be scheduled when the sentinel is ready to shoot.
     */
    Command *const firingCommand;

    aruwsrc::algorithms::OttoBallisticsSolver ballisticsSolver;

    const float userPitchInputScalar;
    const float userYawInputScalar;

    uint32_t prevTime;

    /**
     * Handles scanning logic in the pitch direction
     */
    SetpointScanner pitchScanner;

    /**
     * Handles scanning logic in the yaw direction
     */
    SetpointScanner yawScanner;

    /**
     * A counter that is reset to 0 every time CV starts tracking a target
     * and that keeps track of the number of times `refresh` is called when
     * an aiming solution couldn't be found (either because CV had no target
     * or aiming solution was impossible)
     */
    unsigned int lostTargetCounter = 0;

    /**
     * Yaw and pitch angle increments that the turret will change by each call
     * to refresh when the turret is scanning for a target, in degrees.
     */
    static constexpr float SCAN_DELTA_ANGLE = 0.1f;

    /**
     * The number of times refresh is called without receiving valid CV data to when
     * the command will consider the target lost and start tracking.
     */
    static constexpr int AIM_LOST_NUM_COUNTS = 500;

    /**
     * @return an angle in degrees representing the next "scanning" setpoint
     */
    float scanForTarget(char axis);
};

}  // namespace aruwsrc::control::turret::cv

#endif  // SENTINEL_TURRET_CV_COMMAND_HPP_
