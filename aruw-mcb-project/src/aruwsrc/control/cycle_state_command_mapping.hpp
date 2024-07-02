/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CYCLE_STATE_COMMAND_MAPPING_HPP_
#define CYCLE_STATE_COMMAND_MAPPING_HPP_

#include "tap/control/command_mapping.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::control
{
/**
 * A command mapping that doesn't actually schedule commands. Instead, allows the user to change the
 * state of an object. This can allow you to change the state of a command or subsystem that
 * might be actually mapped to be scheduled by some other remote map state.
 *
 * Each time the remote map state is "pressed" by the user, the state changes and a class-member
 * callback function is called with the new state. The state cycles through a number of states
 * determined by the user.
 *
 * @tparam T The type of state that is being cycled through. Assums that the states to cycle through
 * start at 0 and are integers that increment by 1 between each state.
 * @tparam N The number of states to cycle through.
 * @tparam C The class whose associated update state function will be called when the state has
 * changed.
 */
template <typename T, int N, class C>
class CycleStateCommandMapping : public tap::control::CommandMapping
{
public:
    using StateChangedFn = void (C::*)(T);

    /**
     * Construct a cycle state command mapping wtih some initial state.
     *
     * @param[in] drivers A pointer to the global drivers object.
     * @param[in] rms The map state that will be compared to the actual remote state
     *      to determine whether or not to add `cmds`.
     * @param[in] initialState The initial state that this command mapping should start in.
     * @param[in] stateChangeObject The object whose associated `stateChangedFn` will be called when
     *      the state of this command mapping has changed.
     * @param[in] stateChangedFn Function pointer that must be an instance function of the template
     *      parameter `C`.
     * @param[in] reverseMapState Optional map state that will decrement the state
     */
    CycleStateCommandMapping(
        tap::Drivers *drivers,
        const tap::control::RemoteMapState &rms,
        T initialState,
        C *stateChangeObject,
        StateChangedFn stateChangedFn,
        std::optional<tap::control::RemoteMapState> reverseMapState = std::nullopt)
        : tap::control::CommandMapping(drivers, {}, rms),
          state(initialState),
          stateChangeObject(stateChangeObject),
          stateChangedFn(stateChangedFn),
          reverseMapState(reverseMapState)
    {
    }

    /**
     * Default destructor.
     */
    ~CycleStateCommandMapping() override = default;

    void executeCommandMapping(const tap::control::RemoteMapState &currState) override
    {
        if (mappingSubset(currState) &&
            !(mapState.getNegKeysUsed() && negKeysSubset(mapState, currState)))
        {
            // mapping pressed, state needs updating first time pressed
            if (!pressed)
            {
                state = static_cast<T>((static_cast<int>(state) + 1) % N);
                pressed = true;
                (stateChangeObject->*stateChangedFn)(state);
            }
        }
        else if (
            reverseMapState.has_value() && reverseMapState.value().stateSubsetOf(currState) &&
            !(reverseMapState.value().getNegKeysUsed() &&
              negKeysSubset(reverseMapState.value(), currState)))
        {
            // mapping pressed, state needs updating first time pressed
            if (!pressed)
            {
                int newState = static_cast<int>(state) - 1;
                state = static_cast<T>(newState < 0 ? N - 1 : newState % N);
                pressed = true;
                (stateChangeObject->*stateChangedFn)(state);
            }
        }
        else
        {
            pressed = false;
        }
    }

protected:
    bool pressed = false;
    T state;
    C *stateChangeObject;
    StateChangedFn stateChangedFn;
    std::optional<RemoteMapState> reverseMapState;
};
}  // namespace aruwsrc::control

#endif  // CYCLE_STATE_COMMAND_MAPPING_HPP_
