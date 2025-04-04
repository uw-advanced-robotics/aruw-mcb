/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef LANE_LINE_INDICATOR_HPP_
#define LANE_LINE_INDICATOR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "../projection_utils.hpp"
#include "aruwsrc/algorithms/odometry/transformer_interface.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display
{
using namespace aruwsrc::algorithms::transforms;
using namespace tap::communication::serial;

class LaneLineIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    LaneLineIndicator(
        RefSerialTransmitter &refSerialTransmitter,
        const Transform &worldToCameraTransform);

    void initialize() override final;

    modm::ResumableResult<void> update() override final;

private:
    Transform worldToCameraTransform;
    Tx::Graphic2Message laneLineGraphic;

    static constexpr uint32_t ROBOT_THICKNESS_M = 0.1f;
    static constexpr uint32_t FORWARD_PROJECTION_M = 0.5f;
    static constexpr uint32_t LINE_WIDTH = 1;
};
}  // namespace aruwsrc::control::client_display

#endif
