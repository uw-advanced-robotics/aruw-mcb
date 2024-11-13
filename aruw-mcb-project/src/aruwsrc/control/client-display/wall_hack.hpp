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

#ifndef WALL_HACK_HPP_
#define WALL_HACK_HPP_

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "aruwsrc/algorithms/odometry/transformer_interface.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

using namespace aruwsrc::algorithms::transforms;
using tap::algorithms::CMSISMat;

namespace aruwsrc::control::client_display
{
/**
 * Draws a square where the enemy robot is.
 */
class WallHack : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Construct a WallHack object.
     *
     * @param[in] visionCoprocessor VisionCoprocessor instance.
     * @param[in] refSerialTransmitter RefSerialTransmitter instance.
     * @param[in] transformer TransformerInterface instance.
     */
    WallHack(
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
    static constexpr Tx::GraphicColor COLOR = Tx::GraphicColor::YELLOW;
    static constexpr int SQUARE_SIZE = 30;

    Position enemyPositionWorldFrame, enemyPositionTurretFrame, enemyPositionCameraAxes;
    modm::Vector3f enemyPositionScreenFrame;

    CMSISMat<4, 4> projectionMatrix;

    float horizontalFOV = 135;
    float verticalFOV = 90;

    float near = 0.1, far = 100;

    void setProjectionMatrix();

    modm::Vector3f convertVectorByProjectionMatrix(CMSISMat<3, 1> &vector);

    // clang-format off
    /**
     * Swaps:
     * -y -> x
     *  z -> y
     * -x -> z
     */
    CMSISMat<3, 3> swapAxesMatrix = {
        {0, -1, 0,
         0, 0, 1,
        -1, 0, 0}
    };
    // clang-format on

    uint32_t computedScreenX, computedScreenY;

    // SCREW OZONE DEBUGGING WITH VARIABLES
    struct Vector3
    {
        float x;
        float y;
        float z;
    };
    Vector3 worldFrame, turretFrame, cameraAxes, screenFrame;

    inline void copyToVector3(Vector3 &vec, Position &pos)
    {
        vec.x = pos.x();
        vec.y = pos.y();
        vec.z = pos.z();
    }

    inline void copyToVector3(Vector3 &vec, modm::Vector3f &pos)
    {
        vec.x = pos.x;
        vec.y = pos.y;
        vec.z = pos.z;
    }

};

}  // namespace aruwsrc::control::client_display

#endif  // WALL_HACK_HPP_
