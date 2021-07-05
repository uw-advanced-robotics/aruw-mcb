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

#ifndef ROBOT_CONSTANTS_HPP_
#error "Don't include this file directly, include robot_constants.hpp instead"
#endif

#include "aruwlib/communication/can/can_bus.hpp"
#include "aruwlib/communication/gpio/digital.hpp"

namespace engineer_control::constants
{
namespace motor
{
// CAN 1
// CAN 2
}  // namespace motor

namespace can
{
}  // namespace can

namespace gpio
{
static constexpr aruwlib::gpio::Digital::OutputPin GRABBER_PIN =
    aruwlib::gpio::Digital::OutputPin::E;
static constexpr aruwlib::gpio::Digital::OutputPin X_AXIS_PIN =
    aruwlib::gpio::Digital::OutputPin::F;
static constexpr aruwlib::gpio::Digital::OutputPin TOWER_LEFT_PIN =
    aruwlib::gpio::Digital::OutputPin::G;
static constexpr aruwlib::gpio::Digital::OutputPin TOWER_RIGHT_PIN =
    aruwlib::gpio::Digital::OutputPin::H;
static constexpr aruwlib::gpio::Digital::InputPin TOWER_LEFT_LIMIT_SWITCH =
    aruwlib::gpio::Digital::InputPin::A;
static constexpr aruwlib::gpio::Digital::InputPin TOWER_RIGHT_LIMIT_SWITCH =
    aruwlib::gpio::Digital::InputPin::B;
}  // namespace gpio

namespace chassis
{
}  // namespace chassis
}  // namespace engineer_control::constants

#endif  // ENGINEER_CONSTANTS_HPP_
