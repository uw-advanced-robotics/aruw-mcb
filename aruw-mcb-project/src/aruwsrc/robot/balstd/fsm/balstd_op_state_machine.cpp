/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "balstd_op_state_machine.hpp"

using namespace tap::communication::sensors::imu;

namespace aruwsrc::balstd::fsm
{
BalstdOpStateMachine::BalstdOpStateMachine(
    tap::Drivers* drivers,
    aruwsrc::balstd::chassis::BalstdChassisSubsystem& chassis,
    std::array<
        aruwsrc::balstd::chassis::controllers::BalstdChassisControllerInterface*,
        static_cast<size_t>(BalstdOpState::NUM_STATES)> controllers,
    AbstractIMU& chassisImu,
    aruwsrc::control::buzzer::NoteSequenceCommand* stateTransitionFailChime,
    aruwsrc::control::buzzer::NoteSequenceCommand* watchdogInterventionChime)
    : Subsystem(drivers),
      currentState(BalstdOpState::FALLEN),
      chassis(chassis),
      chassisState(chassis.getChassisState()),
      controllers(controllers),
      chassisImu(chassisImu),
      stateTransitionFailChime(stateTransitionFailChime),
      watchdogInterventionChime(watchdogInterventionChime)
{
}

void BalstdOpStateMachine::initialize()
{
    currentState = BalstdOpState::FALLEN;
    updateState(BalstdOpState::SITTING);  // assumed starting state, must be satisfied on startup
}

void BalstdOpStateMachine::refresh()
{
    if (watchdogTriggered() && currentState != BalstdOpState::FALLEN && isImuCalibrated())
    {
        updateState(BalstdOpState::FALLEN);
        if (watchdogInterventionChime)
            drivers->commandScheduler.addCommand(watchdogInterventionChime);
        return;
    }

    if (disarmRequested && currentState != BalstdOpState::FALLEN)
    {
        updateState(BalstdOpState::SITTING);
        disarmRequested = false;
        return;
    }

    if (getUpRequested)
    {
        if (currentState == BalstdOpState::SITTING && isImuCalibrated())
        {
            updateState(BalstdOpState::BALANCING);
        }
        else if (stateTransitionFailChime)
        {
            drivers->commandScheduler.addCommand(stateTransitionFailChime);
        }
        getUpRequested = false;
        return;
    }
}

bool BalstdOpStateMachine::watchdogTriggered() const
{
    // E-stop if chassis has tilted too much
    if (fabs(chassisState.pitch) > CONTROLLABLE_CHASSIS_PITCH_LIMIT) return true;

    // E-stop if legs have hyperextended
    if (chassisState.leftLegState.qFront > chassisState.leftLegState.config.frontHipInnerLimit ||
        chassisState.leftLegState.qBack < chassisState.leftLegState.config.backHipInnerLimit ||
        chassisState.rightLegState.qFront > chassisState.rightLegState.config.frontHipInnerLimit ||
        chassisState.rightLegState.qBack < chassisState.rightLegState.config.backHipInnerLimit)
        return true;

    return false;
}

}  // namespace aruwsrc::balstd::fsm
