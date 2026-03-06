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

#ifndef FAKE_ENCODER_HPP_
#define FAKE_ENCODER_HPP_

#include "tap/communication/sensors/encoder/wrapped_encoder.hpp"

namespace aruwsrc::communication::sensors::encoder
{
class FakeEncoder : public tap::encoder::WrappedEncoder
{
public:
    FakeEncoder(float fakePosition, float fakeVelocity);

    void initialize() override {}

    bool isOnline() const override;

    inline tap::algorithms::WrappedFloat getPosition() const override
    {
        return tap::algorithms::WrappedFloat(fakePosition, 0, M_TWOPI);
    }

    float getVelocity() const override;

    inline void setFakePosition(float position) { this->fakePosition = position; };

    void setFakeVelocity(float velocity) { this->fakeVelocity = velocity; };

private:
    float fakePosition;
    float fakeVelocity;
    void update();
};

}  // namespace aruwsrc::communication::sensors::encoder

#endif  // ARUW_FAKE_SENSOR_ENCODER_HPP_
