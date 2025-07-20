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

#ifndef BALSTD_OP_STATE_MACHINE_HPP_
#define BALSTD_OP_STATE_MACHINE_HPP_

#include "tap/algorithms/transforms/transform.hpp"
#include "tap/communication/sensors/imu/abstract_imu.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/control/buzzer/note_sequence_command.hpp"
#include "aruwsrc/robot/balstd/chassis/balstd_chassis_state.hpp"
#include "aruwsrc/robot/balstd/chassis/balstd_chassis_subsystem.hpp"
#include "aruwsrc/robot/balstd/chassis/controllers/chassis_controller_interface.hpp"

#include "balstd_op_states.hpp"

namespace aruwsrc::balstd::fsm
{
class BalstdOpStateMachine : public tap::control::Subsystem
{
public:
    BalstdOpStateMachine(
        tap::Drivers* drivers,
        aruwsrc::balstd::chassis::BalstdChassisSubsystem& chassis,
        const std::array<
            aruwsrc::balstd::chassis::controllers::BalstdChassisControllerInterface*,
            static_cast<size_t>(BalstdOpState::NUM_STATES)> controllers,
        tap::communication::sensors::imu::AbstractIMU& chassisImu,
        aruwsrc::control::buzzer::NoteSequenceCommand* stateTransitionFailChime = nullptr,
        aruwsrc::control::buzzer::NoteSequenceCommand* watchdogInterventionChime = nullptr);

    void initialize() override;

    void refresh() override;

    inline const BalstdOpState& getCurrentState() const { return currentState; }

    inline void updateState(BalstdOpState newState)
    {
        if (currentState == newState) return;

        auto* controller = controllers[static_cast<size_t>(newState)];
        if (controller) controller->initialize(chassisState);
        chassis.attachController(controller);
        currentState = newState;
    }

    inline bool isImuCalibrated() const
    {
        return chassisImu.getImuState() ==
               tap::communication::sensors::imu::AbstractIMU::ImuState::IMU_CALIBRATED;
    }

    inline void playChime(aruwsrc::control::buzzer::NoteSequenceCommand* chime)
    {
        if (chime) drivers->commandScheduler.addCommand(chime);
    }

    bool watchdogTriggered() const;

    inline void requestGetUp() { getUpRequested = true; }
    inline void requestDisarm() { disarmRequested = true; }

private:
    BalstdOpState currentState;

    aruwsrc::balstd::chassis::BalstdChassisSubsystem& chassis;
    const aruwsrc::balstd::chassis::BalstdChassisState& chassisState;
    const std::array<
        aruwsrc::balstd::chassis::controllers::BalstdChassisControllerInterface*,
        static_cast<size_t>(BalstdOpState::NUM_STATES)>
        controllers;
    tap::communication::sensors::imu::AbstractIMU& chassisImu;

    aruwsrc::control::buzzer::NoteSequenceCommand* stateTransitionFailChime;
    aruwsrc::control::buzzer::NoteSequenceCommand* watchdogInterventionChime;

    bool getUpRequested{0}, disarmRequested{0};

    static constexpr float CONTROLLABLE_CHASSIS_PITCH_LIMIT = M_PI_4;
};

}  // namespace aruwsrc::balstd::fsm

#endif  // BALSTD_OP_STATE_MACHINE_HPP_