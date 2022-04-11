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

#ifndef TURRET_SUBSYSTEM_CONFIG_HPP_
#define TURRET_SUBSYSTEM_CONFIG_HPP_

#include <cstdint>

namespace aruwsrc::control::turret
{
/**
 * Configuration struct for the turret subsystem.
 */
struct TurretSubsystemConfig
{
    float yawStartAngle = 0;  /// Angle (in degrees) where the turret is assumed to start at. This
                              /// angle value maps to the same value (in encoder ticks) as
                              /// pitchStartEncoderValue.
    uint16_t yawStartEncoderValue = 0;  /// Encoder value between [0, ENC_RESOLUTION) associated
                                        /// with yawStartAngle.
    float yawMinAngle = 0;  /// Min yaw angle that the turret will be limited to (in degrees). This
                            /// value should be between [0, 360) and should be < yawMaxAngle.
    float yawMaxAngle = 0;  /// Max yaw angle that the turret will be limited to (in degrees). This
                            /// value should be between [0, 360) and should be > yawMinAngle.
    float pitchStartAngle = 0;  /// Angle (in degrees) where the turret is assumed to start at. This
                                /// angle value maps to
                                /// the same value (in encoder ticks) as pitchStartEncoderValue
    uint16_t pitchStartEncoderValue = 0;  /// Encoder value between [0, ENC_RESOLUTION) associated
                                          /// with pitchStartAngle.
    float pitchMinAngle = 0;  /// Min pitch angle that the turret will be limited to (in degrees).
                              /// This value should be between [0, 360) and should be < yawMaxAngle.
    float pitchMaxAngle = 0;  /// Max pitch angle that the turret will be limited to (in degrees).
                              /// This value should be between [0, 360) and should be > yawMinAngle.
    bool limitYaw = true;     /// true if the yaw will be limited, false otherwise.
};
}  // namespace aruwsrc::control::turret

#endif  // TURRET_SUBSYSTEM_CONFIG_HPP_
