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
    enum class Mode
    {
        MANUAL = 0,
        BALANCE
    };

    BalstdControlOperatorInterface(tap::Drivers* drivers) : ControlOperatorInterface(drivers) {}

    mockable float getXVel();

    mockable float getYawVel();

    mockable float getRoll();

    mockable float getHeightVel();

    // ====================
    // testing input modes
    // ====================
    /**
     * @return The value used for testing leg VMC movement forward/backward
     */
    mockable float getManualLegXForce();

    /**
     * @return The value used for testing leg VMC up/down movement
     */
    mockable float getManualLegYForce();

    /**
     * @return The value used for testing leg wheel torque
     */
    mockable float getManualWheelTorque();

    /**
     * @return The value used for steering when manual driving
     */
    mockable float getManualSteerTorque();

    inline void setMode(Mode newMode)
    {
        if (mode == newMode) return;
        mode = newMode;
        for (size_t i = 0; i < 6; i++) channelHeldOver[i] = true;
    }

private:
    inline float getRemoteChannel(tap::communication::serial::Remote::Channel channel);

    inline float getModeRestrictedInput(
        tap::communication::serial::Remote::Channel channel,
        Mode mode,
        float max);

    Mode mode;
    bool channelHeldOver[6];

    static constexpr float MAX_X_VEL = 0.4f;         // m/s
    static constexpr float MAX_YAW_VEL = 1.0f;       // rad/s
    static constexpr float MAX_ROLL = M_PI / 6;      // rad
    static constexpr float MAX_HEIGHT_VEL = 0.025f;  // m/s

    static constexpr float MAX_LEG_FORCE = 95.0f;    // N
    static constexpr float MAX_WHEEL_TORQUE = 5.0f;  // N*m
    static constexpr float MAX_STEER_TORQUE = 2.0f;  // N*m
};

}  // namespace aruwsrc::balstd

#endif  // BALSTD_CONTROL_OPERATOR_INTERFACE_HPP__
