/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ENGINEER_CONSTANTS_HPP_
#define ENGINEER_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/communication/gpio/analog.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/motor/dji_motor.hpp"

#include "modm/math/filter/pid.hpp"
#include "modm/math/interpolation/linear.hpp"

namespace aruwsrc::can
{
static constexpr tap::can::CanBus TURRET_MCB_CAN_BUS = tap::can::CanBus::CAN_BUS1;
}  // namespace aruwsrc::can

namespace aruwsrc::serial
{
static constexpr tap::communication::serial::Uart::UartPort VISION_COPROCESSOR_TX_UART_PORT =
    tap::communication::serial::Uart::UartPort::Uart2;

static constexpr tap::communication::serial::Uart::UartPort VISION_COPROCESSOR_RX_UART_PORT =
    tap::communication::serial::Uart::UartPort::Uart3;
}  // namespace aruwsrc::serial

namespace aruwsrc::control::control_operator_interface
{
/**
 * Max acceleration in rpm/s^2 of the chassis in the x direction
 */
static constexpr float MAX_ACCELERATION_X = 10'000.0f;
static constexpr float MAX_DECELERATION_X = 20'000.0f;

/**
 * Max acceleration in rpm/s^2 of the chassis in the y direction
 */
static constexpr float MAX_ACCELERATION_Y = 9'000.0f;
static constexpr float MAX_DECELERATION_Y = 20'000.0f;

/**
 * Max acceleration in rpm/s^2 of the chassis in the r direction
 */
static constexpr float MAX_ACCELERATION_R = 40'000.0f;
static constexpr float MAX_DECELERATION_R = 50'000.0f;
}  // namespace aruwsrc::control::control_operator_interface

#endif  // ENGINEER_CONSTANTS_HPP_
