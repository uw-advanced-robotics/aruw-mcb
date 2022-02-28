/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef VISION_HUD_INDICATORS_HPP_
#define VISION_HUD_INDICATORS_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"
#include "reticle_indicator.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::client_display
{
/**
 * Draws two squares next to the reticle when vision has a target acquired and otherwise hides the
 * squares.
 */
class VisionHudIndicators : public HudIndicator, protected modm::Resumable<3>
{
public:
    /**
     * Construct a VisionHudIndicators object.
     *
     * @param[in] drivers Global drivers instance.
     */
    VisionHudIndicators(aruwsrc::Drivers *drivers);

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

    void initialize() override final;

private:
    static constexpr int VISION_TARGET_FOUND_SQUARE_WIDTH = 15;
    static constexpr int VISION_TARGET_FOUND_X_DISTANCE_FROM_CENTER = 100;
    static constexpr int VISION_TARGET_FOUND_Y_LOCATION = 425;
    static constexpr Tx::GraphicColor VISION_TARGET_FOUND_COLOR = Tx::GraphicColor::GREEN;
    static constexpr uint32_t VISION_TARGET_FOUND_MAX_REFRESH_RATE = 250;

    aruwsrc::Drivers *drivers;

    Tx::Graphic2Message visionTargetFoundGraphics;

    tap::arch::MilliTimeout updateVisionTargetFoundTimeout;

    bool prevVisionTargetStatus = false;
    bool currVisionTargetStatus = false;

    modm::ResumableResult<bool> updateVisionTargetStatus();

    void initializeVisionHudIndicator(Tx::GraphicData *graphicData, int xBoxLocation);
};
}  // namespace aruwsrc::control::client_display

#endif  // VISION_HUD_INDICATORS_HPP_
