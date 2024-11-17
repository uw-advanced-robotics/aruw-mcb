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

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "aruwsrc/algorithms/odometry/transformer_interface.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"
#include "projection_utils.hpp"

using namespace aruwsrc::algorithms::transforms;
using tap::algorithms::CMSISMat;

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
    static constexpr uint16_t WALL_HACK_THICKNESS = 3;

    // Duration of which to show target after losing it
    static constexpr uint16_t TIMEOUT_MS = 5000;
    tap::arch::MilliTimeout targetTimeout;

    static constexpr float SMALL_PLATE_LENGTH_M = 0.135;
    Position plateCornerOffset = Position(0, SMALL_PLATE_LENGTH_M / 2, SMALL_PLATE_LENGTH_M / 2);

    // In world frame
    Position enemyPosition;

    bool square;

    struct ProjectedPlateResult
    {
        bool inFrame;
        // For circle this is center
        uint32_t bottomLeftX;
        uint32_t bottomLeftY;
        // For circle this is radius
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

    ProjectedPlateResult getEnemyPositionPlateSquare(Position enemyPosition)
    {
        ProjectedPlateResult output;

        ProjectedResult enemyPositionScreenFrame =
            convertWorldFrameToScreenFrame(enemyPosition, transformer->getWorldToTurret(0));

        output.inFrame = enemyPositionScreenFrame.inFrame;

        ProjectedResult topRight = convertWorldFrameToScreenFrame(
            enemyPosition,
            transformer->getWorldToTurret(0),
            plateCornerOffset);

        ProjectedResult bottomLeft = convertWorldFrameToScreenFrame(
            enemyPosition,
            transformer->getWorldToTurret(0),
            plateCornerOffset * -1);

        output.bottomLeftX = bottomLeft.screenX;
        output.bottomLeftY = bottomLeft.screenY;
        output.topRightX = topRight.screenX;
        output.topRightY = topRight.screenY;

        return output;
    }

    ProjectedPlateResult getEnemyPositionPlateCircle(Position enemyPosition)
    {
        ProjectedPlateResult output;

        ProjectedResult enemyPositionScreenFrame =
            convertWorldFrameToScreenFrame(enemyPosition, transformer->getWorldToTurret(0));

        output.inFrame = enemyPositionScreenFrame.inFrame;

        ProjectedResult topRight = convertWorldFrameToScreenFrame(
            enemyPosition,
            transformer->getWorldToTurret(0),
            plateCornerOffset);

        // Get magnitude of the vector from the center to the top right corner
        uint32_t difference_x = topRight.screenX - enemyPositionScreenFrame.screenX;
        uint32_t difference_y = topRight.screenY - enemyPositionScreenFrame.screenY;
        float radius = sqrt(difference_x * difference_x + difference_y * difference_y);

        output.bottomLeftX = enemyPositionScreenFrame.screenX;
        output.bottomLeftY = enemyPositionScreenFrame.screenY;
        output.topRightX = (uint32_t)radius;

        return output;
    }
};

}  // namespace aruwsrc::control::client_display

#endif  // VISION_TARGET_INDICATOR_HPP_
