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

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/control/command.hpp"

#include "../algorithms/turret_controller_interface.hpp"
#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"

#include "turret_cv_command_interface.hpp"

using namespace tap::algorithms;
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
class RobotTurretSubsystem;
}

namespace aruwsrc::control::launcher
{
class LaunchSpeedPredictorInterface;
}

namespace aruwsrc::chassis
{
class HolonomicChassisSubsystem;
}

namespace aruwsrc::control::turret::cv
{
/**
 * A command that receives input from the vision system via the `VisionCoprocessor` driver and
 * aims the turret accordingly using a position PID controller.
 *
 * This command, unlike the `SentryTurretCVCommand`, is not responsible for firing projectiles
 * when the auto aim system determines it should fire. Nor does this class scan the turret back and
 * forth.
 *
 * @note If the auto aim system is offline, does not have a target acquired, or has an invalid
 * target (for example, the target is too far away), then user input from the
 * `ControlOperatorInterface` is used to control the turret instead.
 */
class TurretCVCommand : public TurretCVCommandInterface
{
public:
    /**
     * Constructs a TurretCVCommand
     *
     * @param[in] visionCoprocessor Pointer to a global visionCoprocessor object.
     * @param[in] controlOperatorInterface Pointer to a global controlOperatorInterface object.
     * @param[in] turretSubsystem Pointer to the turret to control.
     * @param[in] yawController Pointer to a yaw controller that will be used to control the yaw
     * axis of the turret.
     * @param[in] pitchController Pointer to a pitch controller that will be used to control the
     * pitch axis of the turret.
     * @param[in] ballisticsSolver A ballistics computation engine to use for computing aiming
     * solutions.
     * @param[in] userYawInputScalar When user input is used, this scalar is used to scale the yaw
     * user input.
     * @param[in] userPitchInputScalar When user input is used, this scalar is used to scale the
     * pitch user input.
     * @param[in] turretID The vision turet ID, must be a valid 0-based index, see VisionCoprocessor
     * for more information.
     */
    TurretCVCommand(
        serial::VisionCoprocessor *visionCoprocessor,
        control::ControlOperatorInterface *controlOperatorInterface,
        RobotTurretSubsystem *turretSubsystem,
        algorithms::TurretYawControllerInterface *yawController,
        algorithms::TurretPitchControllerInterface *pitchController,
        aruwsrc::algorithms::OttoBallisticsSolver *ballisticsSolver,
        const float userYawInputScalar,
        const float userPitchInputScalar,
        uint8_t turretID = 0);

    void initialize() override;

    bool isReady() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override;

    const char *getName() const override { return "turret CV"; }

    bool getTurretID() const override { return turretID; }

    /**
     * @return True if vision is active and the turret CV command has acquired the target and the
     * turret is within some tolerance of the target. This tolerance is distance based (the further
     * away the target the closer to the center of the plate the turret must be aiming)
     */
    bool isAimingWithinLaunchingTolerance() const override { return withinAimingTolerance; }

private:
    serial::VisionCoprocessor *visionCoprocessor;
    control::ControlOperatorInterface *controlOperatorInterface;

    uint8_t turretID;

    RobotTurretSubsystem *turretSubsystem;

    algorithms::TurretYawControllerInterface *yawController;
    algorithms::TurretPitchControllerInterface *pitchController;

    aruwsrc::algorithms::OttoBallisticsSolver *ballisticsSolver;

    const float userYawInputScalar;
    const float userPitchInputScalar;

    uint32_t prevTime;

    bool withinAimingTolerance = false;
};
}  // namespace aruwsrc::control::turret::cv

#endif  // TURRET_CV_COMMAND_HPP_
