/*****************************************************************************/
/********** !!! WARNING: CODE GENERATED BY TAPROOT. DO NOT EDIT !!! **********/
/*****************************************************************************/

/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TMOTOR_AK809_HPP_
#define TMOTOR_AK809_HPP_

#include <string>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/can/can_rx_listener.hpp"

#include "../taproot/src/tap/motor/motor_interface.hpp"

namespace aruwsrc
{
class Drivers;
}  // namespace aruwsrc

namespace aruwsrc::motor
{
/**
 * CAN IDs for the feedback messages sent by AK80-9 motor controller. Motor `i` in the set
 * {1, 2,...,8} sends feedback data with in a CAN message with ID 0x200 + `i`.
 * for declaring a new motor, must be one of these motor
 * identifiers
 */
enum TMotorId : uint32_t
{
    MOTOR1 = 0X01,
    MOTOR2 = 0x02,
    MOTOR3 = 0x03,
    MOTOR4 = 0x04,
    MOTOR5 = 0x05,
    MOTOR6 = 0x06,
    MOTOR7 = 0x07,
    MOTOR8 = 0x08,
};

/**
 * A class designed to interface with tmotor brand motors and motor controllers over CAN.
 *
 * @note: the default positive rotation direction (i.e.: when `this->isMotorInverted()
 *      == false`) is counter clockwise when looking at the shaft from the side opposite
 *      the motor.
 *
 *
 * Extends the CanRxListener class to attach a message handler for feedback data from the
 * motor to the CAN Rx dispatch handler.
 *
 * @note Currently there is no error handling for using a motor without having it be properly
 * initialize. You must call the `initialize` function in order for this class to work properly.
 */
class Tmotor_AK809 : public tap::can::CanRxListener, public tap::motor::MotorInterface
{
public:
    // 14-bit encoder
    static constexpr uint16_t ENC_RESOLUTION = 3600;

    /**
     * @param drivers a pointer to the drivers struct
     * @param desMotorIdentifier the ID of this motor controller
     * @param motorCanBus the CAN bus the motor is on
     * @param isInverted if `false` the positive rotation direction of the shaft is
     *      counter-clockwise when looking at the shaft from the side opposite the motor.
     *      If `true` then the positive rotation direction will be clockwise.
     * @param name a name to associate with the motor for use in the motor menu
     * @param encoderWrapped the starting encoderValue to store for this motor.
     *      Will be overwritten by the first reported encoder value from the motor
     * @param encoderRevolutions the starting number of encoder revolutions to store.
     *      See comment for DjiMotor::encoderRevolutions for more details.
     */
    Tmotor_AK809(
        aruwsrc::Drivers* drivers,
        aruwsrc::motor::TMotorId desMotorIdentifier,
        tap::can::CanBus motorCanBus,
        bool isInverted,
        const char* name,
        uint16_t encoderWrapped = ENC_RESOLUTION / 2,
        int64_t encoderRevolutions = 0);

    mockable ~Tmotor_AK809();

    void initialize() override;
    /***
     * @returns Angular position of motor, unwrapped, in 0.1ths of a Degree
     */
    int64_t getEncoderUnwrapped() const override;

    /***
     * @returns Angular position of motor, wrapped from 0 to 3599, in 0.1ths of a Degree
     */
    uint16_t getEncoderWrapped() const override;

    /***
     * @returns Angular position of motor, unwrapped, in radians.
     */
    float getPositionUnwrapped() const
    {
        return ((float)getEncoderUnwrapped()) / ((float)ENC_RESOLUTION) * M_TWOPI;
    };

    /***
     * @returns Angular position of motor, wrapped to one rotation, in radians.
     */
    float getPositionWrapped() const
    {
        return ((float)getEncoderWrapped()) / ((float)ENC_RESOLUTION) * M_TWOPI;
    };

    DISALLOW_COPY_AND_ASSIGN(Tmotor_AK809)

    /**
     * Overrides virtual method in the can class, called every time a message with the
     * CAN message id this class is attached to is received by the can receive handler.
     * Parses the data in the message and updates this class's fields accordingly.
     *
     * @param[in] message the message to be processed.
     */
    void processMessage(const modm::can::Message& message) override;

    /**
     * Set the desired output for the motor. The meaning is a current value between -60 and 60 A, in
     * mA (AKA -60,000 to 60,000)
     *
     * @param[in] desiredOutput the desired motor output. Limited to the range of -60 to 60 A in mA
     *
     */
    void setDesiredOutput(int32_t desiredOutput) override;

    /**
     * @return `true` if a CAN message has been received from the motor within the last
     *      `MOTOR_DISCONNECT_TIME` ms, `false` otherwise.
     */
    bool isMotorOnline() const override;

    /**
     * Serializes send data and deposits it in a message to be sent.
     */
    mockable void serializeCanSendData(modm::can::Message* txMessage) const;

    /**
     * @return the raw `desiredOutput` value which will be sent to the motor controller
     *      (specified via `setDesiredOutput()`)
     */
    int16_t getOutputDesired() const override;

    mockable uint32_t getMotorIdentifier() const;

    /**
     * @return the temperature of the motor as reported by the motor in degrees Celsius
     */
    int8_t getTemperature() const override;

    int16_t getTorque() const override;

    /// For interpreting the sign of return value see class comment
    int16_t getShaftRPM() const override;

    mockable bool isMotorInverted() const;

    mockable tap::can::CanBus getCanBus() const;

    mockable const char* getName() const;

private:
    // wait time before the motor is considered disconnected, in milliseconds
    static const uint32_t MOTOR_DISCONNECT_TIME = 100;

    const char* motorName;

    /**
     * Updates the stored encoder value given a newly received encoder value
     * special logic necessary for keeping track of unwrapped encoder value.
     */
    void updateEncoderValue(uint16_t newEncWrapped);

    aruwsrc::Drivers* drivers;

    uint32_t motorIdentifier;

    tap::can::CanBus motorCanBus;

    int32_t desiredOutput;

    int16_t shaftRPM;

    int8_t temperature;

    int16_t torque;

    /**
     * If `false` the positive rotation direction of the shaft is counter-clockwise when
     * looking at the shaft from the side opposite the motor. If `true` then the positive
     * rotation direction will be clockwise.
     */
    bool motorInverted;

    /**
     * The raw position value reported by the motor controller. It wraps around from
     * {0..3599}, hence "Wrapped"
     */
    float encoderWrapped;

    /**
     * Absolute unwrapped encoder position =
     *      encoderRevolutions * ENCODER_RESOLUTION + encoderWrapped
     * This lets us keep track of some sense of absolute position even while
     * raw encoderValue continuosly loops within {0..8191}. Origin value is
     * arbitrary.
     */
    int64_t encoderRevolutions;

    tap::arch::MilliTimeout motorDisconnectTimeout;
};

}  // namespace aruwsrc::motor

#endif  // TMOTOR_AK809_HPP_
