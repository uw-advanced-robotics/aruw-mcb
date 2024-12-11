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

#ifndef FLAPPY_BIRD_HPP_
#define FLAPPY_BIRD_HPP_

#include "tap/communication/serial/ref_serial.hpp"

#include "aruwsrc/control/client-display/hud_indicator.hpp"
#include "modm/processing/resumable.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{

/**
 * Plays flappy bird
 */

class FlappyBird : public HudIndicator, protected modm::Resumable<3>{



private:
// SCREEN SPACE --------------------------------
static constexpr uint16_t MIN_Y = 180;
static constexpr uint16_t MAX_Y = 825;
static constexpr uint16_t MIN_X = 550;
static constexpr uint16_t MAX_X = 1800;

// BIRD ---------------------------------------
static constexpr uint16_t BIRD_X = MIN_X + 50;
static constexpr uint16_t BIRD_WIDTH_HEIGHT = 16;
static constexpr uint16_t BIRD_LINE_THICKNESS = 15;
static constexpr Tx::GraphicColor BIRD_COLOR = Tx::GraphicColor::WHITE;

// BOUDING LINES -------------------------------
static constexpr uint16_t BOUNDING_LINE_THICKNESS = 10;
static constexpr Tx::GraphicColor TOP_BOUNDING_LINE_COLOR = Tx::GraphicColor::CYAN;
static constexpr Tx::GraphicColor BOTTOM_BOUNDING_LINE_COLOR = Tx::GraphicColor::GREEN;

// PIPES --------------------------------------
static constexpr uint16_t PIPE_WIDTH = 35;
static constexpr uint16_t PIPE_THICKNESS = 100;

// SCORE --------------------------------------
static constexpr uint16_t SCORE_X = MIN_X - 100;
static constexpr uint16_t SCORE_Y = MAX_Y - 50;
static constexpr uint16_t SCORE_SIZE = 100;


}; 

}  // namespace aruwsrc::control::client_display

#endif  // FLAPPY_BIRD_HPP_
