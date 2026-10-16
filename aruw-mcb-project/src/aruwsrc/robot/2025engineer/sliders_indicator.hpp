/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

// does not work rn lol i make attempts to fix soon after midterms :)
#ifndef SLIDERS_INDICATOR_HPP_
#define SLIDERS_INDICATOR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "aruwsrc/control/client-display/indicators/hud_indicator.hpp"
#include "aruwsrc/control/joint/joint_subsystem.hpp"
#include "aruwsrc/robot/2025engineer/engineer_wrist_constants.hpp"
#include "aruwsrc/robot/2025engineer/wrist/wrist_subsystem.hpp"
#include "modm/processing/resumable.hpp"

namespace aruwsrc::engineer
{
using namespace aruwsrc::engineer;
using namespace aruwsrc::engineer::wrist;
class SlidersIndicator : public control::client_display::indicators::HudIndicator,
                         protected modm::Resumable<2>
{
public:
    SlidersIndicator(
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        control::joint::JointSubsystem &gantryLift,
        control::joint::JointSubsystem &gantryExtension,
        control::joint::JointSubsystem &cubeLift,
        WristSubsystem &wristSubsystem,
        wrist::WristConfig wristConfig);

    void initialize() override final;

    modm::ResumableResult<void> sendInitialGraphics() override final;

    modm::ResumableResult<void> update() override final;

private:
    control::joint::JointSubsystem &gantryLift;
    control::joint::JointSubsystem &gantryExtension;
    control::joint::JointSubsystem &cubeLift;
    WristSubsystem &wristSubsystem;
    wrist::WristConfig wristConfig;

    enum class GraphicType : uint8_t
    {
        GANTRY_LIFT = 0,
        GANTRY_EXTENSION = 1,
        CUBE_LIFT = 2,
        WRIST_PITCH = 3,
        WRIST_YAW = 4,
        NUM_GRAPHICS = 5
    };

    static constexpr uint16_t NUM_GRAPHICS = static_cast<uint8_t>(GraphicType::NUM_GRAPHICS);
    static constexpr uint16_t START_Y = 820;
    static constexpr uint16_t START_X = 30;

    static constexpr uint16_t Y_INCREMENT = 80;

    static constexpr uint16_t BOUNDING_BOX_WIDTH = 350;
    static constexpr uint16_t BOUNDING_BOX_HEIGHT = 50;

    // static constexpr uint16_t VER_BOUNDING_BOX_WIDTH = 50;
    // static constexpr uint16_t VER_BOUNDING_BOX_HEIGHT = 350;

    static constexpr uint16_t BOUNDING_BOX_LINE_WIDTH = 10;
    static constexpr uint16_t CIRCLE_SIZE = 13;
    static constexpr uint16_t CIRCLE_LINE_WIDTH = 20;

    static constexpr Tx::GraphicColor GRAPHIC_COLOR =
        Tx::GraphicColor::YELLOW;  // Color of the graphics

    Tx::Graphic7Message sliderOutside;
    Tx::Graphic7Message sliderInside;

    float getPercentage(float value, float minValue, float maxValue) const
    {
        return (value - minValue) / (maxValue - minValue);
    }
};

}  // namespace aruwsrc::engineer

#endif  // SLIDERS_INDICATOR_HPP_
