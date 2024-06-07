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

#ifndef RETICLE_INDICATOR_HPP_
#define RETICLE_INDICATOR_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::control::client_display
{
/**
 * Draws a static reticle defined as a bunch of horizontal lines with a verticle line connecting
 * them. The reticle location is defined by the array
 * `TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES`.
 */
class ReticleIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    /** Number of pixels to offset the reticle from the horizontal center of the screen. */
    static constexpr int RETICLE_CENTER_X_OFFSET = -5;

    /**
     * Construct a ReticleIndicator.
     *
     * @param[in] drivers Global drivers instance.
     */
    ReticleIndicator(
        tap::Drivers &drivers,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

    void initialize() override final;

private:
    /** Line thickness of the reticle, in pixels. */
    static constexpr uint16_t RETICLE_THICKNESS = 1;

    /** Tuple representing a possible horizontal reticle line. The first element is the pixel width
     * of the line, second is Y location of the line (in pixels), third is the color of the reticle
     * line. */
    using ReticleTuple = std::tuple<int16_t, int16_t, Tx::GraphicColor>;

    static constexpr ReticleTuple TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[]{
        ReticleTuple(50, 435, Tx::GraphicColor::YELLOW),  // 1 m
        ReticleTuple(30, 410, Tx::GraphicColor::ORANGE),  // 3 m
        ReticleTuple(10, 370, Tx::GraphicColor::YELLOW),  // 5 m
    };

    /** Size of TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES (so its easier to understand when used
     * in context). */
    static constexpr size_t NUM_RETICLE_COORDINATES =
        MODM_ARRAY_SIZE(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES);
    /** The color of the verticle line that connects the horizontal reticle lines. */
    static constexpr Tx::GraphicColor RETICLE_VERTICAL_COLOR = Tx::GraphicColor::YELLOW;

    tap::Drivers &drivers;

    /**
     * Array of `Graphic5Message`s that will be used to send all of the reticle related graphics.
     * This includes all of the reticle markers from `TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES`
     * plus a verticle line to connect the reticle markers.
     *
     * This is an array of `Graphic5Message`s. There are NUM_RETICLE_COORDINATES + 1 reticle
     * graphics that must be fit into N Graphic5Messages. Each Graphic5Message can hold 5 reticle
     * graphics. Thus there are (NUM_RETICLE_COORDINATES + 1)/5 Graphic5Message structs rounded up
     * to the nearest whole number required to fit the reticle graphics.
     */
    Tx::Graphic5Message
        reticleMsg[tap::algorithms::ceil(static_cast<float>(NUM_RETICLE_COORDINATES + 1) / 5.0f)];

    /** Index used when iterating through the reticleMsg in protothreads. */
    size_t reticleIndex = 0;
};
}  // namespace aruwsrc::control::client_display

#endif  //  RETICLE_INDICATOR_HPP_
