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

#ifndef FSM_EVENT_TRIGGER_COMMAND_HPP_
#define FSM_EVENT_TRIGGER_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "balstd_op_state_machine.hpp"

namespace aruwsrc::balstd::fsm
{
class FSMEventTriggerCommand : public tap::control::Command
{
public:
    using EventFn = void (BalstdOpStateMachine::*)();

    FSMEventTriggerCommand(BalstdOpStateMachine* stateMachine, EventFn event)
        : stateMachine(stateMachine),
          event(event)
    {
        this->addSubsystemRequirement(stateMachine);
    }

    void initialize() override { (stateMachine->*event)(); }

    void execute() override {}

    void end(bool) override {}

    bool isFinished() const override { return true; }

    const char* getName() const override { return "Event Trigger Command"; }

private:
    BalstdOpStateMachine* stateMachine;
    EventFn event;
};  // class FSMEventTriggerCommand

}  // namespace aruwsrc::balstd::fsm
#endif  // FSM_EVENT_TRIGGER_COMMAND_HPP_
