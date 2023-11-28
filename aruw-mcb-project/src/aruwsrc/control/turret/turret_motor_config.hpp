/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_MOTOR_CONFIG_HPP_
#define TURRET_MOTOR_CONFIG_HPP_

#include <cassert>
#include <cstdint>

namespace aruwsrc::control::turret
{
/**
 * Configuration struct for the TurretMotor object.
 */
struct TurretMotorConfig
{
    float startAngle = 0;  /// A reference angle (in radians) to convert to/from encoder ticks. This
                           /// angle value maps to the same value (in encoder ticks) as
                           /// startEncoderValue.
    uint16_t startEncoderValue = 0;  /// Encoder value between [0, ENC_RESOLUTION) associated
                                     /// with startAngle.
    float minAngle = 0;  /// Min angle that the turret will be limited to (in radians). Should be
                         /// within [0, 2 PI) and < minAngle
    float maxAngle = 0;  /// Max wrapped angle that the turret will be limited to (in radians). DOES NOT
                         /// have to be wrapped between [0, 2 PI) to allow for any angle interval,
                         /// but must be > minAngle and <= minAngle + 2 PI.
    bool limitMotorAngles = true;  /// true if the motor will be limited between [minAngle,
                                   /// maxAngle], false otherwise.
};
}  // namespace aruwsrc::control::turret

#endif  // TURRET_MOTOR_CONFIG_HPP_
