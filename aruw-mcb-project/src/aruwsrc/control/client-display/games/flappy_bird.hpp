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

#include "tap/communication/referee/state_hud_indicator.hpp"
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

static constexpr uint16_t MIN_Y = 180;
static constexpr uint16_t MAX_Y = 825;


}; 

}  // namespace aruwsrc::control::client_display

#endif  // FLAPPY_BIRD_HPP_
