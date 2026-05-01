/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef HERO_TURRET_ENCODERS_HPP_
#define HERO_TURRET_ENCODERS_HPP_

#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"
#include "tap/motor/dji_motor_encoder.hpp"

namespace aruwsrc::hero
{
class HeroTurretEncoders
{
public:
    HeroTurretEncoders(
        const tap::encoder::CanEncoder& yawLampreyEncoder,
        const tap::motor::DjiMotorEncoder& yawMotorEncoder)
        : yawLampreyEncoder(yawLampreyEncoder),
          yawMotorEncoder(yawMotorEncoder){

          };
    bool isOnline() const { return yawLampreyEncoder.isOnline() && yawMotorEncoder.isOnline(); }

    // TODO: ask if this is the correct method (wrapped vs unwrapped)
    void setEncoderPosition(float position)
    {
        yawMotorEncoder.getEncoder().setWrappedValue(position);
    }

    float getYawLampreyPosition() { return yawLampreyEncoder.getPosition().getWrappedValue(); }

    float getYawMotorPosition() { return yawMotorEncoder.getEncoder().getWrappedValue(); }

private:
    const tap::encoder::CanEncoder& yawLampreyEncoder;
    const tap::motor::DjiMotorEncoder& yawMotorEncoder;
};
}  // namespace aruwsrc::hero

#endif