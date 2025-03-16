/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "tmotor_ak80_9_encoder.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace aruwsrc::control::motor
{
Tmotor_AK809Encoder::Tmotor_AK809Encoder(
    bool isInverted,
    float gearRatio,
    uint32_t encoderHomePosition)
    : WrappedEncoder(isInverted, ENC_RESOLUTION, gearRatio, encoderHomePosition),
      shaftRPM(0)
{
    encoderDisconnectTimeout.stop();
}

void Tmotor_AK809Encoder::processMessage(const modm::can::Message& message)
{
    encoderDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME);
    shaftRPM = static_cast<int16_t>(message.data[2] << 8 | message.data[3]);  // rpm
    shaftRPM = inverted ? -shaftRPM : shaftRPM;

    uint16_t encoderActual =
        static_cast<uint16_t>(message.data[0] << 8 | message.data[1]);  // encoder value

    updateEncoderValue(encoderActual);
}

bool Tmotor_AK809Encoder::isOnline() const
{
    /*
     * motor online if the disconnect timout has not expired (if it received message but
     * somehow got disconnected) and the timeout hasn't been stopped (initially, the timeout
     * is stopped)
     */
    return !encoderDisconnectTimeout.isExpired() && !encoderDisconnectTimeout.isStopped();
}

float Tmotor_AK809Encoder::getVelocity() const
{
    return this->getShaftRPM() * static_cast<float>(M_TWOPI) / 60.f * this->gearRatio;
}

int16_t Tmotor_AK809Encoder::getShaftRPM() const { return shaftRPM; }
}  // namespace aruwsrc::control::motor
