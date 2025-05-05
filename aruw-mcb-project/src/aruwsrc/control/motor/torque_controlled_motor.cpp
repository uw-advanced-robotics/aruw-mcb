/*
 * Copyright (c) 2020-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/control/motor/torque_controlled_motor.hpp"

namespace aruwsrc::control::motor
{
// Constructors
TorqueControlledMotor::TorqueControlledMotor(
    tap::motor::MotorInterface& motor,
    float kt,
    int32_t maxOutput)
    : motor(motor),
      ktMap({-kt * maxOutput, -maxOutput}, {kt * maxOutput, maxOutput}),
      torqueDesInterpolator(ktMap, 2)
{
}
TorqueControlledMotor::TorqueControlledMotor(
    tap::motor::MotorInterface& motor,
    const modm::Pair<float, float>* in_out_map,
    uint8_t size)
    : motor(motor),
      torqueDesInterpolator(in_out_map, size)
{
}

void TorqueControlledMotor::setTorque(float torque)
{
    TorqueControlledMotor::motor.setDesiredOutput(torqueDesInterpolator.interpolate(torque));
}

inline tap::motor::MotorInterface& TorqueControlledMotor::getMotor() const { return motor; }

};  // namespace aruwsrc::control::motor