/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BARREL_SWITCHER_SUBSYSTEM_HPP
#define BARREL_SWITCHER_SUBSYSTEM_HPP

#include "aruwsrc/control/homeable_subsystem_interface.hpp"

#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::control
{
class BarrelSwitcherSubsystem : public aruwsrc::control::HomeableSubsystemInterface
{
public:
    BarrelSwitcherSubsystem(tap::Drivers* drivers, tap::motor::MotorId motorid);
    void initialize() override;
    void setMotorOutput(int32_t desiredOutput) override;
    bool isStalled() const override;
    void setLowerBound() override;
    void setUpperBound() override;
    
private:
    /**
     * The motor that switches the turret's barrels
    */
   tap::motor::DjiMotor motor;

    /**
     * upper bound for motor's encoder
     * note: the lower bound is 0
    */
    int32_t motorUpperBound;
};
} //namespace aruwsrc::control
#endif