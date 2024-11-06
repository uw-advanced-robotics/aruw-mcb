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

#ifndef DAMAGE_INDICATOR_HPP_
#define DAMAGE_INDICATOR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "aruwsrc/algorithms/plate_hit_tracker.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display
{

/**
 * Draws 1 line on the screen around the center circle to indicate where damage has come from.
 */
class DamageIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Construct a DamageIndicator object.
     *
     * @param[in] plateHitTracker plateHitTracker instance.
     * @param[in] refSerialTransmitter RefSerialTransmitter instance.
     */
    DamageIndicator(
        aruwsrc::algorithms::PlateHitTracker &plateHitTracker,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    void initialize() override final;

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

private:
    static constexpr uint16_t DAMAGE_INDICATOR_THICKNESS = 20;
    static constexpr uint16_t DAMAGE_INDICATOR_LENGTH = 20;

    static constexpr uint16_t DISTANCE_FROM_CENTER = 60;

    static constexpr uint16_t LINE_LENGTH = 20;

    static constexpr uint16_t X_POS = SCREEN_WIDTH / 2;
    static constexpr uint16_t Y_POS = SCREEN_HEIGHT / 2;


    Tx::Graphic1Message damageGraphic;

    aruwsrc::algorithms::PlateHitTracker &plateHitTracker;

    static constexpr float DAMAGE_THRESHOLD = 0.1;

    static constexpr float INDICATOR_OFFSET_RADIANS = modm::toRadian(90);

    // DEBUG
    float x,y;
    float degree;
    float degreeRadian;
    bool hasValidAngle;
    aruwsrc::algorithms::PlateHitTracker::PlateHitBinData peakAngleBin;

};

}  // namespace aruwsrc::control::client_display

#endif  // DAMAGE_INDICATOR_HPP_
