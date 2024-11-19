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

#ifndef VISION_TARGET_INDICATOR_HPP_
#define VISION_TARGET_INDICATOR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "aruwsrc/algorithms/odometry/transformer_interface.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"
#include "projection_utils.hpp"

using namespace aruwsrc::algorithms::transforms;

namespace aruwsrc::control::client_display
{
/**
 * Draws a square where the enemy robot is.
 */
class VisionTargetIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    VisionTargetIndicator(
        aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        TransformerInterface *transformer);

    void initialize() override final;

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

private:
    aruwsrc::serial::VisionCoprocessor &visionCoprocessor;
    TransformerInterface *transformer;

    Tx::Graphic1Message visionTargetGraphic;
    static constexpr uint16_t INDICATOR_LINE_THICKNESS = 3;

    RefSerialData::Tx::GraphicColor INDICATOR_COLOR =
        RefSerialData::Tx::GraphicColor::GREEN;

    static constexpr float SMALL_PLATE_LENGTH_M = 0.135;
    Position plateCornerOffset = Position(0, SMALL_PLATE_LENGTH_M / 2, SMALL_PLATE_LENGTH_M / 2);

    // In world frame
    Position enemyPosition;

    struct ProjectedPlateResult
    {
        bool inFrame;
        uint32_t bottomLeftX;
        uint32_t bottomLeftY;
        uint32_t topRightX;
        uint32_t topRightY;
        Position enemyCenterCameraFrame;

        ProjectedPlateResult()
            : inFrame(false),
              bottomLeftX(0),
              bottomLeftY(0),
              topRightX(0),
              topRightY(0),
              enemyCenterCameraFrame(0, 0, 0)
        {
        }
    };

    ProjectedPlateResult enemyPositionScreenFrame;

    ProjectedPlateResult getEnemyPositionPlateSquare(Position enemyPositionWorldFrame)
    {
        ProjectedPlateResult output;

        Position cameraFrame =
            transformer->getWorldToTurret(0).apply(enemyPositionWorldFrame) + VTM_OFFSET;

        ProjectedResult screenFrame = convertCameraFrameToScreenFrame(cameraFrame);

        output.inFrame = screenFrame.inFrame;

        ProjectedResult topRight = convertCameraFrameToScreenFrame(cameraFrame + plateCornerOffset);

        ProjectedResult bottomLeft =
            convertCameraFrameToScreenFrame(cameraFrame + (plateCornerOffset * -1));

        output.bottomLeftX = bottomLeft.screenX;
        output.bottomLeftY = bottomLeft.screenY;
        output.topRightX = topRight.screenX;
        output.topRightY = topRight.screenY;

        return output;
    }
};

}  // namespace aruwsrc::control::client_display

#endif  // VISION_TARGET_INDICATOR_HPP_
