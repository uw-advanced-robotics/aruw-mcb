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

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::turret
{
class TurretSubsystem;
}

namespace aruwsrc::control::turret::cv
{
/**
 * A command that receives input from the vision system via the `LegacyVisionCoprocessor` driver and
 * aims the turret accordingly using a position PID controller.
 *
 * This command, unlike the `SentinelTurretCVCommand`, is not responsible for firing projectiles
 * when the auto aim system determines it should fire.
 */
class TurretCVCommand : public tap::control::Command
{
public:
    /**
     * @param[in] drivers Pointer to a global drivers object.
     * @param[in] turretSubsystem Pointer to the turret to control.
     * @param[in] yawController Pointer to a yaw controller that will be used to control the yaw
     * axis of the turret.
     * @param[in] pitchController Pointer to a pitch controller that will be used to control the
     * pitch axis of the turret.
     */
    TurretCVCommand(
        aruwsrc::Drivers *drivers,
        TurretSubsystem *turretSubsystem,
        algorithms::TurretYawControllerInterface *yawController,
        algorithms::TurretPitchControllerInterface *pitchController);

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

    uint32_t prevTime;
};  // class TurretCvCommand

}  // namespace aruwsrc::control::turret::cv

#endif  // TURRET_CV_COMMAND_HPP_
