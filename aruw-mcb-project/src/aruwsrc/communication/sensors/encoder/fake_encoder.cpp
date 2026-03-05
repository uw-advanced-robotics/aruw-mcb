/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/communication/sensors/encoder/fake_encoder.hpp"

namespace aruwsrc::communication::sensors::encoder
{
FakeEncoder::FakeEncoder(float fakePosition, float fakeVelocity)
    : WrappedEncoder(false, 10000),
      fakePosition(fakePosition),
      fakeVelocity(fakeVelocity)
{
}

bool FakeEncoder::isOnline() const { return true; }

tap::algorithms::WrappedFloat FakeEncoder::getPosition() const
{
    return WrappedEncoder::getPosition();
}

float FakeEncoder::getVelocity() const { return WrappedEncoder::getVelocity(); }

}  // namespace aruwsrc::communication::sensors::encoder
