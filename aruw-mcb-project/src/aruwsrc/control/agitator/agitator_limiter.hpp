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

#ifndef AGITATOR_LIMITER_HPP_
#define AGITATOR_LIMITER_HPP_

#include "aruwsrc/drivers.hpp"
#include "agitator_subsystem.hpp"
#include "../launcher/friction_wheel_subsystem.hpp"

namespace aruwsrc
{
class Drivers;

namespace control::agitator 
{

/**
 * CHANGE !!!!!!!!!!!!!!!!!!!!!1
 */
class AgitatorLimiter : public aruwsrc::agitator::AgitatorSubsystem
{
public:
    /**
     * @param[in] drivers A pointer to the `Drivers` struct.
     * @param[in] agitatorSubsystem 
     * @param[in] frictionWheelSubsystem
     * 
     */
    AgitatorLimiter(
        aruwsrc::Drivers* drivers,
        const tap::algorithms::SmoothPidConfig& pidParams,
        float agitatorGearRatio,
        tap::motor::MotorId agitatorMotorId,
        tap::can::CanBus agitatorCanBusId,
        bool isAgitatorInverted,
        float jammingDistance,
        uint32_t jammingTime,
        bool jamLogicEnabled,
        aruwsrc::control::launcher::FrictionWheelSubsystem* frictionWheelSubsystem);

    void refresh() override;

private:
    /**
     * The object that runs jam detection.
     */
    tap::control::setpoint::SetpointContinuousJamChecker jamChecker;

    /**
     * Stores the jam state of the subsystem
     */
    bool subsystemJamStatus = false;
    
    aruwsrc::control::launcher::FrictionWheelSubsystem* frictionWheelSubsystem;
  
}; // class AgitatorLimiter
} // namespace control::agitator
} // namespace aruwsrc

#endif  // AGITATOR_LIMITER_HPP_
