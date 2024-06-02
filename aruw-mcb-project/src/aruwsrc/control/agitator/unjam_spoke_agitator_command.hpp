/*
* Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef UNJAM_SPOKE_AGITATOR_COMMAND_HPP_
#define UNJAM_SPOKE_AGITATOR_COMMAND_HPP_

#include <cstdint>

#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"

#include "tap/control/setpoint/interfaces/integrable_setpoint_subsystem.hpp"

namespace aruwsrc::control::agitator
{
/**
 * Command that unjams a velocity setpoint subsystem corresponding to a 2023/2024 spoke agitator.
 * 
 * Runs the agitator backwards until either reaching a @param targetUnjamIntegralChange or when the
 * @param maxWaitTime has past. Then, it attempts to rotate back to the original position. If it
 * successfully reaches that position, the agitator is considered unjammed and the command ends.
 * Otherwise, it repeats this process up to a maximum of @param targetCycleCount times before
 * giving up and ending.
 */
class UnjamSpokeAgitatorCommand : public tap::control::Command
{
public:
    /// Config struct that the user passes into the UnjamSpokeAgitatorCommand's constructor.
    struct Config
    {
        /**
         * The target integral setpoint from the current integral value with units `units * seconds`
         * (units of setpoint integrated with respect to time) that the integral setpoint subsystem
         * will move back and forth by with unjamming.
         *
         * @attention This value must be positive and > 0.
         */
        float targetUnjamIntegralChange;
        /**
         * The target setpoint in units that the integral setpoint subsystem will move
         * back and forth at.
         *
         * @attention This value must be positive and > 0.
         */
        float unjamSetpoint;
        /**
         * The maximum amount of time the controller will wait for the subsystem to reach
         * targetUnjamIntegralChange in milliseconds before trying to move in the opposite
         * direction.
         *
         * @attention This value must be > 1000 * (targetUnjamIntegralChange / unjamSetpoint) since
         * this is the minimum possible time it will take for the motor to rotate
         * targetUnjamIntegralChange units.
         */
        uint32_t maxWaitTime;
        /**
         * The number of cycles to attempt to rotate the velocity setpoint subsystem back and
         * forth.
         *
         * @attention This value must be positive and > 0.
         */
        uint16_t targetCycleCount;
    };

    /**
     * @param[in] integrableSetpointSubsystem The associated agitator subsystem to control.
     */
    UnjamSpokeAgitatorCommand(
        tap::control::setpoint::IntegrableSetpointSubsystem& integrableSetpointSubsystem,
        const Config& config);

    bool isReady() override;

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "unjam spoke agitator"; }

private:
    enum UnjamState
    {
        UNJAM_BACKWARD,  ///< The subsystem is being commanded backwards
        RETURN_FORWARD,   ///< The subsystem is being commanded forwards
        JAM_CLEARED,     ///< The jam is cleared, the subsystem is no longer being told to move.
    };

    tap::control::setpoint::IntegrableSetpointSubsystem& integrableSetpointSubsystem;

    Config config;

    /**
     * Timeout for time allowed to rotate past the `unjamThreshold`.
     */
    tap::arch::MilliTimeout unjamRotateTimeout;

    /**
     * counts the number of times the subsystem has been commanded backwards
     */
    uint16_t backwardsCount;

    UnjamState currUnjamState;

    float positionBeforeUnjam;

    void beginUnjamForwards();

    void beginUnjamBackwards();
};  // class UnjamSpokeAgitatorCommand

}  // namespace aruwsrc::control::agitator

#endif  // UNJAM_SPOKE_AGITATOR_COMMAND_HPP_
