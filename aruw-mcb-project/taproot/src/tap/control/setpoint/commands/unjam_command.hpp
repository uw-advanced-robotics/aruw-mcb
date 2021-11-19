/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef AGITATOR_UNJAM_COMMAND_HPP_
#define AGITATOR_UNJAM_COMMAND_HPP_

#include <cstdint>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

namespace tap
{
namespace control
{
namespace setpoint
{
// Forward declarations
class SetpointSubsystem;

/**
 * Command that takes control of a setpoint subsystem moves it back and forth.
 * One back and forward motion counts as a cycle. Unjamming cycles start by trying
 * to move in negative direction before trying to move in positive direction.
 *
 * If successful the unjam command will return the setpoint of the subsystem back
 * to its original value. If not successful, setpoint is set to current value so as
 * to not damage motors.
 *
 * Like most setpoint commands this one will not schedule/will deschedule if setpointSubsystem
 * goes offline.
 */
class UnjamCommand : public tap::control::Command
{
public:
    /**
     * @param[in] setpointSubsystem The associated agitator subsystem to control.
     * @param[in] unjamDisplacement How far to attempt to displace the subsystem
     *      during an unjam. This value should be positive! Absolute value will be
     *      taken if negative.
     * @param[in] unjamThreshold The minimum displacement to be reached both
     *      forwards and backwards before the subsystem is considered unjammed.
     *      This value must be positive. Absolute value will be taken
     *      if negative.
     * @param[in] maxWaitTime The maximum amount of time the controller will
     *      wait for the subsystem to reach unjam target in milliseconds before
     *      trying to move in the opposite direction.
     * @param[in] targetCycleCount the number of cycles to attempt to wiggle the subsystem
     */
    UnjamCommand(
        SetpointSubsystem* setpointSubsystem,
        float unjamDisplacement,
        float unjamThreshold,
        uint32_t maxWaitTime,
        uint_fast16_t targetCycleCount);

    bool isReady() override;

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "agitator unjam"; }

private:
    enum UnjamState
    {
        UNJAM_BACKWARD,
        UNJAM_FORWARD
    };

    void beginUnjamForwards();

    void beginUnjamBackwards();

    /**
     * Timeout for time allowed to rotate past the `unjamThreshold`.
     */
    tap::arch::MilliTimeout unjamRotateTimeout;

    /**
     * Maximum time the command will spend trying to reach unjam target in
     * one direction before giving up and trying other direction.
     */
    const uint32_t maxWaitTime;

    SetpointSubsystem* setpointSubsystem;

    /**
     * The target displacement in both directions during unjam
     */
    float unjamDisplacement;

    /**
     * The minimum displacement in both directions at which point jam is
     * considered cleared
     */
    float unjamThreshold;

    /**
     * The number of times the comand will try to wiggle the subsystem.
     */
    const uint_fast16_t targetCycleCount;

    /**
     * counts the number of times the subsystem has been commanded backwards
     */
    uint_fast16_t backwardsCount;

    UnjamState currUnjamState;

    float setpointBeforeUnjam;

    float valueBeforeUnjam;

    bool backwardsCleared;

    bool forwardsCleared;
};  // class UnjamCommand

}  // namespace setpoint

}  // namespace control

}  // namespace tap

#endif  // AGITATOR_UNJAM_COMMAND_HPP_
