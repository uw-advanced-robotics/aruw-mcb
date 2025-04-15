/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TORQUE_CONTROLLED_MOTOR_HPP_
#define TORQUE_CONTROLLED_MOTOR_HPP_

#include "tap/motor/motor_interface.hpp"

#include "modm/math/interpolation/linear.hpp"

namespace aruwsrc::control::motor
{
class TorqueControlledMotor
{
public:
    TorqueControlledMotor(tap::motor::MotorInterface& motor, float kt, int32_t maxOutput)
        : motor(motor),
          torqueDesInterpolator(
              {
                  {-maxOutput, -kt * maxOutput},
                  {maxOutput, kt * maxOutput},
              },
              2)
    {
    }

    void setTorque(const float torque) { motor.setDesiredOutput(torque); }

    inline tap::motor::MotorInterface& getMotor() const { return motor; }

private:
    tap::motor::MotorInterface& motor;

    modm::interpolation::Linear<modm::Pair<float, float>> torqueDesInterpolator;
};

}  // namespace aruwsrc::control::motor

#endif  // TORQUE_CONTROLLED_MOTOR_HPP_