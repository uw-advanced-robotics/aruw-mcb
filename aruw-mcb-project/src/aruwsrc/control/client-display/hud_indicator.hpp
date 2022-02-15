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

#ifndef HUD_INDICATOR_HPP_
#define HUD_INDICATOR_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "modm/processing/resumable.hpp"

namespace aruwsrc::control::client_display
{
/**
 * Macro that should be used in a resumable function. Blocks (in a resumable fashion) for some time
 * based on the associated graphic. After sending a graphic via ref serial, you should call this
 * macro and pass in a pointer to the graphic you just sent via serial. Delaying avoids overwhelming
 * the serial line.
 *
 * @param[in] graphic Pointer to graphic data that was just sent.
 */
#define DELAY_REF_GRAPHIC(graphic)                                                 \
    delayTimer.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(graphic)); \
    RF_WAIT_UNTIL(delayTimer.isExpired() || delayTimer.isStopped());

class HudIndicator : protected modm::Resumable<2>,
                     protected tap::communication::serial::RefSerialData
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

    virtual modm::ResumableResult<bool> sendInitialGraphics() = 0;

    virtual modm::ResumableResult<bool> update() = 0;

    virtual void initialize() = 0;

    /**
     * Resets the list name generator so the next time it is queried via `getUnusedListName`, the
     * function returns {0, 0, 0}.
     */
    static void resetListNameGenerator();

protected:
    /**
     * Graphics must have unique 3 byte names. Utility function for getting a list name that is
     * currently unused. Use this function exclusively to avoid graphic name clashes.
     *
     * If no list names are available (all are in use), won't set the listName and will raise an
     * error.
     *
     * @param[out] listName Array to put an unused list name in.
     */
    static void getUnusedListName(uint8_t listName[3]);

    static uint32_t currListName;

    /**
     * Timer used to delay between sending messages to the referee system.
     */
    tap::arch::MilliTimeout delayTimer;
};
}  // namespace aruwsrc::control::client_display

#endif  // HUD_INDICATOR_HPP_
