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

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display::indicators
{
class PumpIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    PumpIndicator(
        aruwsrc::algorithms::PlateHitTracker &plateHitTracker,
        const aruwsrc::control::turret::RobotTurretSubsystem &turretSubsystem,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    void initialize() override final;

    modm::ResumableResult<void> update() override final;

private:
    aruwsrc::algorithms::PlateHitTracker &plateHitTracker;
    const aruwsrc::control::turret::RobotTurretSubsystem &turretSubsystem;

    static constexpr uint16_t PUMP_INDICATOR_DIAMETER = 20;


    static constexpr uint16_t X_POS = SCREEN_WIDTH / 2;
    static constexpr uint16_t Y_POS = SCREEN_HEIGHT / 2;

    static constexpr float INDICATOR_OFFSET_RADIANS = modm::toRadian(90);

    Tx::Graphic1Message damageGraphic;

    static constexpr float CENTER_THRESHOLD = modm::toRadian(40);

    tap::arch::MilliTimeout decayTimeout;
    static constexpr uint32_t DECAY_TIMEOUT_MILLIS = 5000;

    float x, y;
    float hitAngleRadian = 0;

    aruwsrc::algorithms::PlateHitTracker::PlateHitBinData peakAngleBin;

    uint32_t prevTimestamp;
};

}  // namespace aruwsrc::control::client_display::indicators

#endif  // DAMAGE_INDICATOR_HPP_
