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

#ifndef FRICTION_WHEEL_LUT_FINDER_COMMAND_HPP_
#define FRICTION_WHEEL_LUT_FINDER_COMMAND_HPP_

#include "tap/control/command.hpp"

namespace aruwsrc::control::launcher
{
class FrictionWheelInterface;

/** Command to generate a lookup table between friction wheel RPM and projectile launch speed. */
class FrictionWheelLUTFinderCommand : public tap::control::Command
{
public:
    FrictionWheelLUTFinderCommand(FrictionWheelInterface *subsystem);

    bool isReady() override { return true; };

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char *getName() const override { return "friction wheel LUT finder command"; }

private:
    FrictionWheelInterface *subsystem;
    uint32_t prevTime = 0;
    uint16_t curRPM = 0;

    static constexpr uint16_t TIME_INC_MILLI = 500;
    static constexpr uint16_t RPM_MAX = 8000;
    static constexpr uint8_t RPM_INCREMENT = 100;

};  // class FrictionWheelLUTFinderCommand

}  // namespace aruwsrc::control::launcher

#endif  // FRICTION_WHEEL_TEST_COMMAND_HPP_
