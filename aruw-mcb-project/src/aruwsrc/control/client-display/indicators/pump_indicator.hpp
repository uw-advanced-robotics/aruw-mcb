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

#ifndef PUMP_INDICATOR_HPP_
#define PUMP_INDICATOR_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "aruwsrc/control/digital/digital_out_subsystem.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display::indicators
{
class PumpIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    PumpIndicator(
        tap::communication::serial::RefSerialTransmitter& refSerialTransmitter,
        aruwsrc::control::digital::DigitalOutSubsystem& subsystem);

    void initialize() override final;

    modm::ResumableResult<void> update() override final;

private:
    aruwsrc::control::digital::DigitalOutSubsystem& subsystem;
    Tx::Graphic1Message pumpGraphic;

    static constexpr uint16_t PUMP_INDICATOR_RADIUS = 20;

    static constexpr uint16_t Y_POS = 778;
    static constexpr uint16_t X_POS = 79;
};

}  // namespace aruwsrc::control::client_display::indicators

#endif  // PUMP_INDICATOR_HPP_