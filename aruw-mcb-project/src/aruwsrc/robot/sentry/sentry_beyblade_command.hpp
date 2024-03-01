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

#ifndef SENTRY_BEYBLADE_COMMAND_HPP_
#define SENTRY_BEYBLADE_COMMAND_HPP_

#include "tap/algorithms/ramp.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/swerve_chassis_subsystem.hpp"
#include "aruwsrc/control/turret/turret_motor.hpp"
#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"
#include "aruwsrc/robot/sentry/sentry_transforms.hpp"

namespace aruwsrc::sentry
{
/**
 * A command that automatically rotates the chassis while maintaining turret angle
 */
class SentryBeybladeCommand : public tap::control::Command
{
public:
    struct SentryBeybladeConfig
    {
        /**
         * Fraction of max chassis speed that will be applied to rotation when beyblading
         */
        const float beybladeRotationalSpeedFractionOfMax;
        /**
         * Fraction between [0, 1], what we multiply user translational input by when beyblading.
         */
        const float beybladeTranslationalSpeedMultiplier;
        /**
         * The fraction to cut rotation speed while moving and beyblading
         */
        const float beybladeRotationalSpeedMultiplierWhenTranslating;
        /**
         * Threshold, a fraction of the maximum translational speed that is used to determine if
         * beyblade speed should be reduced (when translating at an appreciable speed beyblade speed
         * is reduced).
         */
        const float translationalSpeedThresholdMultiplierForRotationSpeedDecrease;
        /**
         * Rotational speed to update the beyblade ramp target by each iteration until final
         * rotation setpoint reached, in RPM.
         */
        const float beybladeRampRate;
    };

    SentryBeybladeCommand(
        tap::Drivers* drivers,
        aruwsrc::chassis::SwerveChassisSubsystem* chassis,
        const aruwsrc::control::turret::TurretMotor* yawMotor,
        aruwsrc::control::sentry::SentryControlOperatorInterface& operatorInterface,
        const tap::algorithms::transforms::Transform& worldToChassis,
        const SentryBeybladeConfig config);

    /**
     * Sets rotational input target on Ramp
     */
    void initialize() override;

    /**
     * Updates rotational speed with ramp, translates and rotates x and y inputs based on turret
     * angle Sets chassis motor outputs
     */
    void execute() override;

    void end(bool) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "chassis beyblade"; }

private:
    

    float rotationDirection;

    tap::algorithms::Ramp rotateSpeedRamp;

    tap::Drivers* drivers;
    aruwsrc::chassis::HolonomicChassisSubsystem* chassis;
    const aruwsrc::control::turret::TurretMotor* yawMotor;
    aruwsrc::control::sentry::SentryControlOperatorInterface& operatorInterface;
    const tap::algorithms::transforms::Transform& worldToChassis;
    const SentryBeybladeConfig config;
};  // class BeybladeCommand

}  // namespace aruwsrc::sentry

#endif  // BEYBLADE_COMMAND_HPP_
