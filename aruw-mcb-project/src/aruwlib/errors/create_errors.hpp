/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CREATE_ERRORS_HPP_
#define CREATE_ERRORS_HPP_

#include "aruwlib/Drivers.hpp"

#include "system_error.hpp"

namespace aruwlib
{
namespace errors
{
/**
 * Example for how to create and add an error to the `ErrorController` in the
 * instance of the `Drivers` class passed in:
 * ```cpp
 * RAISE_ERROR(
 *     drivers, "Error in DJI Serial",
 *     aruwlib::errors::Location::DJI_SERIAL,
 *     aruwlib::errors::ErrorType::INVALID_CRC);
 * ```
 * then call ErrorController::updateLedDisplay() to display the error via the LEDs
 */
#define RAISE_ERROR(drivers, desc, l, et)                                          \
    do                                                                             \
    {                                                                              \
        aruwlib::errors::SystemError stringError(desc, __LINE__, __FILE__, l, et); \
        drivers->errorController.addToErrorList(stringError);                      \
    } while (0);

}  // namespace errors

}  // namespace aruwlib

#endif  // CREATE_ERRORS_HPP_
