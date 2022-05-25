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

#ifndef HUD_INDICATOR_HPP_
#define HUD_INDICATOR_HPP_

#include <array>
#include <optional>

#include "tap/architecture/timeout.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "modm/processing/resumable.hpp"

namespace tap::communication::serial
{
class RefSerialTransmitter;
}

namespace aruwsrc::control::client_display
{
/**
 * A generic HUD indicator interface class with helper utilities that other HUD indicators may use.
 */
class HudIndicator : protected tap::communication::serial::RefSerialData
{
public:
    /*
     * Note that absolute X/Y pixel coordinates are measured from the bottom left side of the
     * screen, X is increasing from left to right and Y is increasing from bottom to top.
     */

    /** Width of the screen, in pixels. */
    static constexpr uint16_t SCREEN_WIDTH = 1920;
    /** Height of the screen, in pixels. */
    static constexpr uint16_t SCREEN_HEIGHT = 1080;

    /** Unless you have a particular reason to do otherwise, place all graphics in this layer. */
    static constexpr uint8_t DEFAULT_GRAPHIC_LAYER = 0;
    /** The line spacing of the characters in a characterGraphicMessage if a `\n` character is
     * inserted. */
    static constexpr float CHARACTER_LINE_SPACING = 1.5f;

    HudIndicator(tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    virtual modm::ResumableResult<bool> sendInitialGraphics() = 0;

    virtual modm::ResumableResult<bool> update() = 0;

    virtual void initialize() = 0;

    /**
     * Resets the graphic name generator so the next time it is queried via `getUnusedGraphicName`,
     * the function returns {0, 0, 0}.
     */
    static void resetGraphicNameGenerator();

protected:
    /**
     * Largest graphic name possible that getUnusedGraphicName will return
     */
    static constexpr uint32_t MAX_GRAPHIC_NAME = 0xffffff;

    /**
     * Graphics must have unique 3 byte names. Utility function for getting a graphic name that is
     * currently unused. Use this function exclusively to avoid graphic name clashes.
     *
     * If no list names are available (all are in use), won't set the graphicName.
     *
     * @param[out] graphicName Array to put an unused list name in.
     *
     */
    static std::optional<std::array<uint8_t, 3>> getUnusedGraphicName();

    static uint32_t currGraphicName;

    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter;

    /**
     * Timer used to delay between sending messages to the referee system.
     */
    tap::arch::MilliTimeout delayTimer;
};
}  // namespace aruwsrc::control::client_display

#endif  // HUD_INDICATOR_HPP_
