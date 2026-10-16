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

#ifndef TMOTOR_AK80_9_ENCODER_HPP_
#define TMOTOR_AK80_9_ENCODER_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/communication/sensors/encoder/encoder_interface.hpp"
#include "tap/util_macros.hpp"

#include "modm/architecture/interface/can_message.hpp"
#include "modm/math/geometry/angle.hpp"

namespace aruwsrc::control::motor
{
/**
 * A class designed to interface with the encoder for DJI brand motors and motor controllers over
 * CAN. This includes the C610 and C620 motor controllers and the GM6020 motor (that has a built-in
 * motor controller).
 *
 * @note: the default positive rotation direction (i.e.: when `this->isMotorInverted()
 *      == false`) is counter clockwise when looking at the shaft from the side opposite
 *      the motor. This is specified in the C620 user manual (page 18).
 *
 * DJI motor encoders store a consistent encoding for a given angle across power-cycles.
 * This means the encoder angle reported by the motor can have meaning if the encoding
 * for an angle is unique as it is for the GM6020s. However for geared motors like the
 * M3508 where a full encoder revolution does not correspond 1:1 to a shaft revolution,
 * it is impossible to know the orientation of the shaft given just the encoder value.
 *
 * Combining them with some form of absolute encoder on the output shaft would give you knowledge of
 * the orientation of the output shaft.
 */
class Tmotor_AK809Encoder : public tap::encoder::EncoderInterface
{
public:
    // 3600 ticks w/ 1:9 gear reduction (hacky ik)
    static constexpr uint16_t ENC_RESOLUTION = 3600 / 9;

    /***
     * WARNING! The Ak80-9 initializes it's encoder position to 1750 on boot.
     */

    /***
     * WARNING! the AK80-9 outputs position in the range [-32000, 32000]
     * If the output shaft rotates more than this, then the output value will saturate! The motor
     * continues to track it's position but you will not get the values over CAN. Try to not
     * overrotate the motor.
     */

    /**
     * @param isInverted if `false` the positive rotation direction of the shaft is
     *      counter-clockwise when looking at the shaft from.
     *      If `true` then the positive rotation direction will be clockwise.
     * @param gearRatio the ratio of input revolutions to output revolutions of this encoder.
     * @param encoderHomePosition the zero position for the encoder in encoder ticks.
     */
    Tmotor_AK809Encoder(
        bool isInverted,
        float gearRatio = 1,
        uint16_t encoderHomePosition = 0);  // 1750

    void initialize() override{};

    void alignWith(EncoderInterface* other) override;

    void resetEncoderValue() override;

    bool isOnline() const override;

    inline tap::algorithms::WrappedFloat getPosition() const override;

    float getVelocity() const override;

    /**
     * The current RPM reported by the motor controller.
     */
    mockable int16_t getShaftRPM() const;

    DISALLOW_COPY_AND_ASSIGN(Tmotor_AK809Encoder)

    /**
     * Overrides virtual method in the can class, called every time a message with the
     * CAN message id this class is attached to is received by the can receive handler.
     * Parses the data in the message and updates this class's fields accordingly.
     *
     * @param[in] message the message to be processed.
     */
    mockable void processMessage(const modm::can::Message& message);

private:
    // wait time before the motor is considered disconnected, in milliseconds
    static const uint32_t MOTOR_DISCONNECT_TIME = 100;

    tap::arch::MilliTimeout encoderDisconnectTimeout;

    const uint32_t encoderResolution;

    /**
     * The raw position received from the encoder.
     */
    int32_t rawPosition;

    /**
     * The current encoder position in ticks.
     */
    int32_t positionTicks;

    /**
     * The encoder position converted into output rotations
     */
    tap::algorithms::WrappedFloat position;

    bool inverted;

    const float gearRatio;

    /**
     * The actual encoder wrapped value received from CAN messages where this motor
     * is considered to have an encoder value of 0. encoderHomePosition is 0 by default.
     */
    int16_t encoderHomePosition;

    int16_t shaftRPM;

    void updateEncoderValue(int16_t encoderActual);
};

}  // namespace aruwsrc::control::motor

#endif  // TMOTOR_AK80_9_ENCODER_HPP_
