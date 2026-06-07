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
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor_encoder.hpp"

#include "aruwsrc/communication/sensors/encoder/fake_encoder.hpp"

namespace aruwsrc::hero
{
class HeroTurretEncoderSubsystem : public tap::control::Subsystem
{
public:
    HeroTurretEncoderSubsystem(
        tap::Drivers* drivers,
        tap::encoder::CanEncoder& yawLampreyEncoder,
        tap::encoder::CanEncoder& yawCanEncoder,
        tap::encoder::EncoderInterface* yawMotorEncoder,
        const float finalHomeOffset)
        : tap::control::Subsystem(drivers),
          yawLampreyEncoder(yawLampreyEncoder),
          yawCanEncoder(yawCanEncoder),
          yawMotorEncoder(yawMotorEncoder),
          fakeLampreyEncoder(0, 0),
          finalHomeOffset(finalHomeOffset)
    {
    }
    const char* getName() const { return "Hero Turret Encoder Subsystem"; }

    bool isOnline() const
    {
        return yawLampreyEncoder.isOnline() && yawCanEncoder.isOnline() &&
               yawMotorEncoder->isOnline();
    }

    void initialize()
    {
        yawLampreyEncoder.initialize();
        yawCanEncoder.initialize();
    }

    void setEncoderPosition(float position)
    {
        fakeLampreyEncoder.setFakePosition(position - finalHomeOffset);
        yawMotorEncoder->alignWith(&fakeLampreyEncoder);
    }

    float getYawLampreyPosition() { return yawLampreyEncoder.getPosition().getWrappedValue(); }

    float getYawMotorPosition() { return yawMotorEncoder->getPosition().getWrappedValue(); }

    float getYawEncoderPosition() { return yawCanEncoder.getPosition().getWrappedValue(); }

private:
    tap::encoder::CanEncoder& yawLampreyEncoder;
    tap::encoder::CanEncoder& yawCanEncoder;
    tap::encoder::EncoderInterface* yawMotorEncoder;

    aruwsrc::communication::sensors::encoder::FakeEncoder fakeLampreyEncoder;

    float finalHomeOffset;
};
}  // namespace aruwsrc::hero

#endif