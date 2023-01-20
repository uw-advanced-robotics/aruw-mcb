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

#include "low_battery_buzzer_command.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc::communication
{
LowBatteryBuzzerCommand::LowBatteryBuzzerCommand(
    aruwsrc::control::buzzer::BuzzerSubsystem& buzzer,
    aruwsrc::Drivers* drivers)
    : buzzer(buzzer),
      drivers(drivers)
{
    addSubsystemRequirement(&buzzer);
}

void LowBatteryBuzzerCommand::initialize() {}

void LowBatteryBuzzerCommand::execute()
{
    if (drivers->refSerial.getRobotData().chassis.volt < LOW_BATTERY_THRESHOLD)
    {
        buzzer.playNoise();
    }
    else
    {
        buzzer.stop();
    }
}

void LowBatteryBuzzerCommand::end(bool) { buzzer.stop(); }

}  // namespace aruwsrc::communication