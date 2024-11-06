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
 * Draws 3 lines on the screen to indicate where damage is coming from.
 * Either left, right, or bottom of center circle.
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

    static constexpr uint16_t DAMAGE_INDICATOR_LR_Y_BOTTOM =
        SCREEN_HEIGHT / 2 - DAMAGE_INDICATOR_LENGTH / 2;
    static constexpr uint16_t DAMAGE_INDICATOR_LR_Y_TOP =
        DAMAGE_INDICATOR_LR_Y_BOTTOM + DAMAGE_INDICATOR_LENGTH;
    static constexpr uint16_t DAMAGE_INDICATOR_LEFT_X = 900;
    static constexpr uint16_t DAMAGE_INDICATOR_RIGHT_X = 1000;


    static constexpr uint16_t DAMAGE_INDICATOR_BOTTOM_X = SCREEN_WIDTH / 2;
    static constexpr uint16_t DAMAGE_INDICATOR_BOTTOM_Y_BOTTOM = 480;
    static constexpr uint16_t DAMAGE_INDICATOR_BOTTOM_Y_TOP =
        DAMAGE_INDICATOR_BOTTOM_Y_BOTTOM + DAMAGE_INDICATOR_LENGTH;

    Tx::Graphic1Message leftGraphic, rightGraphic, bottomGraphic;

    aruwsrc::algorithms::PlateHitTracker &plateHitTracker;
};

}  // namespace aruwsrc::control::client_display

#endif  // DAMAGE_INDICATOR_HPP_
