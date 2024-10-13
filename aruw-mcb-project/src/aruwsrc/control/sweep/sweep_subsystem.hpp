/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef SWEEP_SUBSYSTEM_HPP_
#define SWEEP_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::control::sweep
{

class SweepSubsystem : public tap::control::Subsystem
{
public:
    SweepSubsystem(tap::Drivers* drivers, tap::motor::DjiMotor* motor)
        : tap::control::Subsystem(drivers),
          motor(motor)
    {
    }

    void initialize() override { motor->initialize(); }

    void refresh() override {}

    void refreshSafeDisconnect() override { motor->setDesiredOutput(0); }

    const char* getName() const override { return "Sweep Subsystem"; }

    tap::motor::DjiMotor* motor;

};  // class SweepSubsystem

}  // namespace aruwsrc::control::sweep
#endif  // SWEEP_SUBSYSTEM_HPP_
