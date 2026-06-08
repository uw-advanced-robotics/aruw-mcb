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

#ifndef CYCLE_STATE_MODE_CONTROLLER_HPP_
#define CYCLE_STATE_MODE_CONTROLLER_HPP_

namespace aruwsrc::control
{
/**
 * @tparam T The type of state that is being cycled through. Assums that the states to cycle through
 * start at 0 and are integers that increment by 1 between each state.
 * @tparam N The number of states to cycle through.
 * @tparam C The class whose associated update state function will be called when the state has
 * changed.
 */
template <typename T, int N, class C>
class CycleStateModeController
{
public:
    using StateChangedFn = void (C::*)(T);

    /**
     * Construct a cycle state mode controller wtih some initial state.
     * @param[in] initialState The initial state that this controller should start in.
     * @param[in] stateChangeObject The object whose associated `stateChangedFn` will be called when
     *      the state of the controller has changed.
     * @param[in] stateChangedFn Function pointer that must be an instance function of the template
     *      parameter `C`.
     */
    explicit CycleStateModeController(
        T initialState,
        C *stateChangeObject,
        StateChangedFn stateChangedFn)
        : state(initialState),
          stateChangeObject(stateChangeObject),
          stateChangedFn(stateChangedFn)
    {
        (stateChangeObject->*stateChangedFn)(state);
    }

    void cycleState()
    {
        counter++;
        state = static_cast<T>((static_cast<int>(state) + 1) % N);
        (stateChangeObject->*stateChangedFn)(state);
    }

    void reverseCycleState()
    {
        counter--;
        int newState = static_cast<int>(state) - 1;
        state = static_cast<T>(newState < 0 ? N - 1 : newState % N);
        (stateChangeObject->*stateChangedFn)(state);
    }

    T getCurrentState() const { return state; }

private:
    int counter = 0;
    T state;
    C *stateChangeObject;
    StateChangedFn stateChangedFn;
};
}  // namespace aruwsrc::control

#endif