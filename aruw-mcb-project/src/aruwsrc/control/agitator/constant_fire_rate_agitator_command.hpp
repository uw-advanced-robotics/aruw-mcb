/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CONSTANT_FIRE_RATE_AGITATOR_COMMAND_HPP_
#define CONSTANT_FIRE_RATE_AGITATOR_COMMAND_HPP_

#include "aruwsrc/control/agitator/constant_velocity_agitator_command.hpp"
#include "aruwsrc/control/agitator/fire_rate_reselection_manager_interface.hpp"

namespace aruwsrc::control::agitator
{
/**
 * A command that runs in one of two modes:
 * 1) Constant-velocity mode for higher shot rates.
 * 2) Normal move-integral mode for low shot rates.
 */
class ConstantFireRateAgitatorCommand : public ConstantVelocityAgitatorCommand
{
public:
    struct Config
    {
        tap::control::setpoint::MoveIntegralCommand::Config moveIntegralConfig;
        float targetShotRateRps;
        int agitatorPocketCount;
        float minConstantVelocityRpm;
        FireRateReselectionManagerInterface* fireRateReselectionManager = nullptr;
    };

    ConstantFireRateAgitatorCommand(
        tap::control::setpoint::IntegrableSetpointSubsystem& integrableSetpointSubsystem,
        const Config& config)
        : ConstantVelocityAgitatorCommand(integrableSetpointSubsystem, config.moveIntegralConfig),
          constantVelocitySetpoint(
              config.targetShotRateRps *
              (M_TWOPI / static_cast<float>(config.agitatorPocketCount))),
          slowSetpoint(config.moveIntegralConfig.desiredSetpoint),
          fireRateReselectionManager(config.fireRateReselectionManager),
          agitatorPocketCount(config.agitatorPocketCount),
          minConstantVelocityRpm(config.minConstantVelocityRpm),
          targetVelocityRpm(
              config.targetShotRateRps * 60.0f / static_cast<float>(config.agitatorPocketCount))
    {
    }

    const char* getName() const override { return "constant fire rate agitator command"; }

    void setFireRateReselectionManager(FireRateReselectionManagerInterface* manager)
    {
        fireRateReselectionManager = manager;
    }

    void initialize() override
    {
        if (shouldUseManagerDrivenFireRate())
        {
            updateTargetVelocityFromManager();
        }
        const bool useSlowMoveIntegral = shouldUseSlowMoveIntegral();
        config.desiredSetpoint = useSlowMoveIntegral ? slowSetpoint : constantVelocitySetpoint;
        enableConstantRotation(!useSlowMoveIntegral);
        ConstantVelocityAgitatorCommand::initialize();
    }

    void execute() override
    {
        if (!shouldUseManagerDrivenFireRate())
        {
            return;
        }

        updateTargetVelocityFromManager();
        if (!shouldUseSlowMoveIntegral())
        {
            config.desiredSetpoint = constantVelocitySetpoint;
            integrableSetpointSubsystem.setSetpoint(config.desiredSetpoint);
        }
    }

private:
    bool shouldUseManagerDrivenFireRate() const
    {
        return fireRateReselectionManager != nullptr &&
               fireRateReselectionManager->getFireRateReadinessState() ==
                   FireRateReadinessState::READY_USE_RATE_LIMITING;
    }

    void updateTargetVelocityFromManager()
    {
        if (fireRateReselectionManager == nullptr)
        {
            return;
        }

        const float targetShotRateRps = fireRateReselectionManager->getFireRateRps();
        constantVelocitySetpoint =
            targetShotRateRps * (M_TWOPI / static_cast<float>(agitatorPocketCount));
        targetVelocityRpm = targetShotRateRps * 60.0f / static_cast<float>(agitatorPocketCount);
    }

    bool shouldUseSlowMoveIntegral() const { return targetVelocityRpm < minConstantVelocityRpm; }

    float constantVelocitySetpoint;
    float slowSetpoint;
    FireRateReselectionManagerInterface* fireRateReselectionManager;
    int agitatorPocketCount;
    float minConstantVelocityRpm;
    float targetVelocityRpm;
};
}  // namespace aruwsrc::control::agitator

#endif  // CONSTANT_FIRE_RATE_AGITATOR_COMMAND_HPP_
