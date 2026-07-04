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

#ifndef PUMP_INDICATOR_HPP_
#define PUMP_INDICATOR_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "aruwsrc/algorithms/plate_hit_tracker.hpp"
#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"
#include "modm/processing/resumable.hpp"
#include "aruwsrc/control/digital/DigitalOutSubsystem"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display::indicators
{
class PumpIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    PumpIndicator(
        aruwsrc::algorithms::PlateHitTracker &plateHitTracker,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    void initialize() override final;

    modm::ResumableResult<void> update() override final;

private:
    aruwsrc::algorithms::PlateHitTracker &plateHitTracker;
    aruwsrc::control::digital::DigitalOutSubsystem& subsystem;


    static constexpr uint16_t PUMP_INDICATOR_RADIUS = 20;

    static constexpr uint16_t Y_POS = 865;
    static constexpr uint16_t X_POS = 123;
};

}  // namespace aruwsrc::control::client_display::indicators

#endif  // DAMAGE_INDICATOR_HPP_
