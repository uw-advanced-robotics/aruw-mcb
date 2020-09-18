/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef AGITATOR_UNJAM_COMMAND_HPP_
#define AGITATOR_UNJAM_COMMAND_HPP_

#include <aruwlib/Drivers.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/control/command.hpp>
#include <aruwlib/motor/dji_motor.hpp>

#include "agitator_subsystem.hpp"

namespace aruwsrc
{
namespace agitator
{
/**
 * Command that takes control of an agitator motor and attempts to unjam it. Whether
 * or not the agitator is actually in a jam condition is not up for this command to
 * determine. It is assumed that unjamming must occur.
 */
class AgitatorUnjamCommand : public aruwlib::control::Command
{
public:
    /**
     * @param[in] agitator The associated agitator subsystem to control.
     * @param[in] agitatorMaxUnjamAngle The maximum backwards rotation of the agitator
     *      to be used in an unjam step. A random backwards angle is subsequently choosen
     *      each time the agitator unjam command attempts to rotate the agitator backwards.
     * @param[in] agitatorMaxWaitTime The maximum amount of time the controller will
     *      wait for the motor to rotate backwards before commencing with a forward rotation.
     */
    AgitatorUnjamCommand(
        AgitatorSubsystem* agitator,
        float agitatorMaxUnjamAngle,
        uint32_t agitatorMaxWaitTime = AGITATOR_MAX_WAIT_TIME);

    /**
     * Restarts the timer that will used to determine when to rotate the agitator forward,
     * defines a random unjam angle between `[MIN_AGITATOR_UNJAM_ANGLE, agitatorUnjamAngleMax]`.
     * Then a desired agitator angle is set based on the random angle calculated and the agitator's
     * current position. The `salvationTimeout` is additionally restarted. This will be used to
     * rapidly rotate the agitator backwards an entire rotation if the timeout expires while still
     * attempting to unjam.
     */
    void initialize() override;

    /**
     * Checks the salvation timer and goes into salvation mode if the timer has expired. Then
     * checks the state of the unjam sequence. If in salvation mode, wait until the rotate timeout
     * has expired (which is reset if in salvation mode to be `SALVATION_TIMEOUT_MS`). If in unjam
     * back mode, check if the agitator unjam back timer has expired or if the agitator has reached
     * the desired unjam back position and move into the reset state. If in the reset state, attempt
     * to set the agitator's desired angle back to where it was to start. If the agitator fails to
     * reach the position in some time, start the unjam process over again.
     */
    void execute() override;

    /**
     * No-op
     */
    void end(bool interrupted) override;

    /**
     * @return `true` if the current unjam state is `FINISHED`, `false` otherwise.
     */
    bool isFinished() const override;

    const char* getName() const override { return "agitator unjam command"; }

private:
    static constexpr uint32_t SALVATION_TIMEOUT_MS = 2000;

    static constexpr uint32_t SALVATION_UNJAM_BACK_WAIT_TIME = 1000;

    static constexpr float AGITATOR_SETPOINT_TOLERANCE = aruwlib::algorithms::PI / 16.0f;

    /**
     * The maximum time that the command will wait from commanding the agitator to rotate
     * backwards to rotating forwards again.
     */
    static constexpr uint32_t AGITATOR_MAX_WAIT_TIME = 130;

    /**
     * Minimum angle the agitator will rotate backwards when unjamming.
     */
    static constexpr float MIN_AGITATOR_UNJAM_ANGLE = aruwlib::algorithms::PI / 4.0f;

    enum AgitatorUnjamState
    {
        AGITATOR_SALVATION_UNJAM_BACK,
        AGITATOR_UNJAM_BACK,
        AGITATOR_UNJAM_RESET,
        FINISHED
    };

    AgitatorUnjamState currUnjamstate;

    /**
     * Time allowed to rotate back the the `currAgitatorUnjamAngle`.
     */
    aruwlib::arch::MilliTimeout agitatorUnjamRotateTimeout;

    aruwlib::arch::MilliTimeout salvationTimeout;

    /**
     * Usually set to `AGITATOR_MAX_WAIT_TIME`, but can be user defined.
     */
    uint32_t agitatorMaxWaitTime;

    AgitatorSubsystem* connectedAgitator;

    float agitatorUnjamAngleMax;

    float currAgitatorUnjamAngle;

    float agitatorSetpointBeforeUnjam;
};  // class AgitatorUnjamCommand

}  // namespace agitator

}  // namespace aruwsrc

#endif  // AGITATOR_UNJAM_COMMAND_HPP_
