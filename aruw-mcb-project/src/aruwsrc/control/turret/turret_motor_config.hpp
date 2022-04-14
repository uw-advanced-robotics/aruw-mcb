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

#include <cstdint>

namespace aruwsrc::control::turret
{
/**
 * Configuration struct for the TurretMotor object.
 */
struct TurretMotorConfig
{
    float startAngle = 0;  /// Angle (in radians) where the turret is assumed to start at. This
                           /// angle value maps to the same value (in encoder ticks) as
                           /// startEncoderValue.
    uint16_t startEncoderValue = 0;  /// Encoder value between [0, ENC_RESOLUTION) associated
                                     /// with startAngle.
    float minAngle = 0;  /// Min angle that the turret will be limited to (in radians). If this
                         /// valid is < maxAngle, limiting will be applied normally. If it is >
                         /// maxAngle, it is assumed that the range of valid turret angles is
                         /// between [minAngle, maxAngle] and doesn't pass through 0.
    float maxAngle = 0;  /// Max angle that the turret will be limited to (in radians). This
                         /// value should be > minAngle.
    bool limitMotorAngles =
        true;  /// true if the motor will be limited between [minAngle, maxAngle], false otherwise
};
}  // namespace aruwsrc::control::turret

#endif  // TURRET_MOTOR_CONFIG_HPP_
