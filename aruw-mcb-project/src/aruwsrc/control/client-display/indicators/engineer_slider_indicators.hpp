/*
 * Copyright (c) 2026-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <cmath>

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "aruwsrc/communication/can/aruw_pressure_sensor.hpp"
#include "aruwsrc/control/client-display/indicators/hud_indicator.hpp"
#include "aruwsrc/control/joint/joint_subsystem.hpp"
#include "aruwsrc/robot/engineer/cube_storage/cube_storage_subsystem.hpp"
#include "modm/processing/resumable.hpp"

namespace aruwsrc::control::client_display::indicators
{
using namespace aruwsrc::engineer;
using namespace aruwsrc::communication::can;

class EngineerSliderIndicators : public HudIndicator, protected modm::Resumable<2>
{
public:
    EngineerSliderIndicators(
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        const joint::JointSubsystem &extension,
        const cube_storage::CubeStorageSubsystem &cubeStorageSubsystem,
        AruwPressureSensor &wristPressureSensor,
        AruwPressureSensor &cubeStoragePressureSensor1,
        AruwPressureSensor &cubeStoragePressureSensor2);

    void initialize() override final;

    modm::ResumableResult<void> sendInitialGraphics() override final;

    modm::ResumableResult<void> update() override final;

private:
    const joint::JointSubsystem &extension;
    const cube_storage::CubeStorageSubsystem &cubeStorageSubsystem;
    AruwPressureSensor &wristPressureSensor;
    AruwPressureSensor &cubeStoragePressureSensor1;
    AruwPressureSensor &cubeStoragePressureSensor2;

    enum class GraphicType : uint8_t
    {
        EXTENSION_LINE,
        EXTENSION_CIRCLE,
        CUBE_STORAGE_LINE,
        CUBE_STORAGE_WRIST,
        CUBE_STORAGE_1,
        CUBE_STORAGE_2,
        NUM_INDICATORS
    };

    static constexpr uint16_t NUM_GRAPHICS = static_cast<uint8_t>(GraphicType::NUM_INDICATORS);

    static constexpr uint16_t EXTENSION_START_X = 100;
    static constexpr uint16_t EXTENSION_Y = 300;
    static constexpr uint16_t EXTENSION_END_X = 300;
    static constexpr uint16_t EXTENSION_WIDTH = EXTENSION_END_X - EXTENSION_START_X;
    static constexpr uint16_t EXTENSION_MIDDLE_X = (EXTENSION_START_X + EXTENSION_END_X) / 2;

    static constexpr uint16_t CUBE_STORAGE_START_X = 100;
    static constexpr uint16_t CUBE_STORAGE_Y = 500;
    static constexpr uint16_t CUBE_STORAGE_END_X = 300;
    static constexpr uint16_t CUBE_STORAGE_WIDTH = CUBE_STORAGE_END_X - CUBE_STORAGE_START_X;
    static constexpr uint16_t CUBE_STORAGE_MIDDLE_X =
        (CUBE_STORAGE_START_X + CUBE_STORAGE_END_X) / 2;

    static constexpr float CUBE_STORAGE_OFFSET_PERCENT_1 = 0.2f;
    static constexpr float CUBE_STORAGE_OFFSET_PERCENT_2 = -0.2f;

    static constexpr uint16_t LINE_WIDTH = 10;
    static constexpr uint16_t CIRCLE_SIZE = 15;

    static constexpr Tx::GraphicColor LINE_COLOR =
        Tx::GraphicColor::YELLOW;  // Color of the line graphics

    static constexpr Tx::GraphicColor EXTENSION_CIRCLE_COLOR =
        Tx::GraphicColor::ORANGE;  // Color of the extension circle graphic

    static constexpr Tx::GraphicColor HAS_CUBE_COLOR =
        Tx::GraphicColor::CYAN;  // Color of the cube storage wrist position graphic

    static constexpr Tx::GraphicColor NO_CUBE_COLOR =
        Tx::GraphicColor::PINK;  // Color of the cube storage wrist position graphic when no cube is
                                 // present

    Tx::Graphic7Message sliders;

    float percentage(float value, float minValue, float maxValue) const
    {
        return (value - minValue) / (maxValue - minValue);
    }
};

}  // namespace aruwsrc::control::client_display::indicators

#endif  // SLIDERS_INDICATOR_HPP_
