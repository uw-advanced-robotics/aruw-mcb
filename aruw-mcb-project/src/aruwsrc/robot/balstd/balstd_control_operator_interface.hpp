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

#ifndef BALSTD_CONTROL_OPERATOR_INTERFACE_HPP_
#define BALSTD_CONTROL_OPERATOR_INTERFACE_HPP_

#include "aruwsrc/robot/control_operator_interface.hpp"

namespace aruwsrc::balstd
{
class BalstdControlOperatorInterface : public aruwsrc::control::ControlOperatorInterface
{
public:
    using Channel = tap::communication::serial::Remote::Channel;

    enum class Mode
    {
        MANUAL = 0,
        BALANCE
    };

    enum class Input
    {
        X_VEL = 0,
        YAW_VEL,
        ROLL,
        HEIGHT_VEL,
        MANUAL_LEG_X_FORCE,
        MANUAL_LEG_Y_FORCE,
        MANUAL_WHEEL_TORQUE,
        MANUAL_STEER_TORQUE,
        MANUAL_GRAV_COMP,
        NUM_INPUTS,
    };

    struct InputConfig
    {
        Channel channel;
        Mode mode;
        float max;
    };

    BalstdControlOperatorInterface(tap::Drivers* drivers) : ControlOperatorInterface(drivers) {}

    template <Input I>
    inline float getInput()
    {
        constexpr Channel channel = INPUTS[static_cast<size_t>(I)].channel;
        constexpr Mode mode = INPUTS[static_cast<size_t>(I)].mode;
        constexpr float max = INPUTS[static_cast<size_t>(I)].max;
        return getModeRestrictedInput(channel, mode, max);
    }

    inline void setMode(Mode newMode)
    {
        if (mode == newMode) return;
        mode = newMode;
        for (size_t i = 0; i < 5; i++) channelHeldOver[i] = true;
    }

private:
    inline float getRemoteChannel(Channel channel);

    float getModeRestrictedInput(Channel channel, Mode mode, float max);

    Mode mode;
    bool channelHeldOver[5];

    static constexpr InputConfig INPUTS[static_cast<size_t>(Input::NUM_INPUTS)]{
        {
            // x vel
            .channel = Channel::LEFT_VERTICAL,
            .mode = Mode::BALANCE,
            .max = 0.4f,  // m/s
        },
        {
            // yaw vel
            .channel = Channel::RIGHT_HORIZONTAL,
            .mode = Mode::BALANCE,
            .max = -1.0f,  // rad/s
        },
        {
            // roll
            .channel = Channel::LEFT_HORIZONTAL,
            .mode = Mode::BALANCE,
            .max = -M_PI / 6,  // rad
        },
        {
            // height vel
            .channel = Channel::WHEEL,
            .mode = Mode::BALANCE,
            .max = -0.025f,  // m/s
        },
        {
            // leg force x
            .channel = Channel::LEFT_HORIZONTAL,
            .mode = Mode::MANUAL,
            .max = 95.0f,  // N
        },
        {
            // leg force y
            .channel = Channel::LEFT_VERTICAL,
            .mode = Mode::MANUAL,
            .max = -95.0f,  // N
        },
        {
            // wheel torque
            .channel = Channel::RIGHT_VERTICAL,
            .mode = Mode::MANUAL,
            .max = 5.0f,  // N*m
        },
        {
            // steer torque
            .channel = Channel::RIGHT_HORIZONTAL,
            .mode = Mode::MANUAL,
            .max = 2.0f,  // N*m
        },
        {
            // manual grav comp
            .channel = Channel::WHEEL,
            .mode = Mode::MANUAL,
            .max = 59.0f,  // N
        },
    };
};

}  // namespace aruwsrc::balstd

#endif  // BALSTD_CONTROL_OPERATOR_INTERFACE_HPP__
