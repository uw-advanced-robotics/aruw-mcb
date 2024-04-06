/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_MINOR_CONTROL_COMMAND_HPP_
#define TURRET_MINOR_CONTROL_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"

namespace aruwsrc
{
class Drivers;
}  // namespace aruwsrc

namespace aruwsrc::control::turret::sentry
{
/**
 * Command that takes user input from the `SentryControlOperatorInterface` to control the pitch and
 * yaw axis of some turret turret minor using some passed in yaw and pitch controller upon
 * construction.
 */
class TurretMinorSentryControlCommand : public tap::control::Command
{
public:
    /**
     * @param[in] drivers Pointer to a global drivers object.
     * @param[in] yawController Reference to a yaw controller that will be used to control the yaw
     * axis of the turret.
     * @param[in] turretMinorSubsystem Reference to the sentry turret to control.
     * @param[in] pitchController Reference to a pitch controller that will be used to control the
     * pitch axis of the turret.
     * @param[in] userYawInputScalar Value to scale the user input from `ControlOperatorInterface`
     * by. Basically mouse sensitivity.
     * @param[in] userPitchInputScalar See userYawInputScalar.
     */
    TurretMinorSentryControlCommand(
        tap::Drivers *drivers,
        aruwsrc::control::sentry::SentryControlOperatorInterface &controlOperatorInterface,
        aruwsrc::control::sentry::SentryTurretMinorSubsystem &turretMinorSubsystem,
        algorithms::TurretYawControllerInterface &yawController,
        algorithms::TurretPitchControllerInterface &pitchController,
        float userYawInputScalar,
        float userPitchInputScalar);

    bool isReady() override;

    const char *getName() const override { return "User turret minor control"; }

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override;

private:
    tap::Drivers *drivers;
    aruwsrc::control::sentry::SentryControlOperatorInterface &controlOperatorInterface;

    aruwsrc::control::sentry::SentryTurretMinorSubsystem &turretMinorSubsystem;

    uint32_t prevTime = 0;

    algorithms::TurretYawControllerInterface &yawController;
    algorithms::TurretPitchControllerInterface &pitchController;

    const float userYawInputScalar;
    const float userPitchInputScalar;
};
}  // namespace aruwsrc::control::turret::sentry

#endif  // TURRET_MINOR_CONTROL_COMMAND_HPP_
