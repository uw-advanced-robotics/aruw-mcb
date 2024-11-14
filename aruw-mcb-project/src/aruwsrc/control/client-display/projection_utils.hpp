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

#pragma once

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/transforms/transform.hpp"

#include "hud_indicator.hpp"

using namespace tap::algorithms;
using namespace tap::algorithms::transforms;
using aruwsrc::control::client_display::HudIndicator;

namespace aruwsrc::control::client_display
{

static constexpr int HORIZONTAL_FOV = 169;
static constexpr int VERTICAL_FOV = 160;

static constexpr float NEAR_CUTOFF_M = 0.1;
static constexpr float FAR_CUTOFF_M = 100;

// clang-format off
static const CMSISMat<3, 3> worldAxesToCameraAxes = {
    {0, -1, 0,
     0, 0, 1,
    -1, 0, 0}
};
// clang-format on

struct ProjectedResult
{
    bool inFrame;
    uint32_t screenX;
    uint32_t screenY;
};

#if defined(TARGET_STANDARD_CYGNUS)
static Position VTM_OFFSET_FRAME = Position(0, -0.05, -0.1);
#else
static Position VTM_OFFSET_FRAME = Position(0, 0, 0);
#endif

/**
 * Creates a projection matrix.
 */
static CMSISMat<4, 4> getProjectionMatrix()
{
    float horizontalScale = 1.0f / tanf(HORIZONTAL_FOV * 0.5 * M_PI / 180);
    float verticalScale = 1.0f / tanf(VERTICAL_FOV * 0.5 * M_PI / 180);

    CMSISMat<4, 4> projectionMatrix;
    projectionMatrix.data[0] = horizontalScale;
    projectionMatrix.data[4 * 1 + 1] = verticalScale;

    projectionMatrix.data[4 * 2 + 2] = -FAR_CUTOFF_M / (FAR_CUTOFF_M - NEAR_CUTOFF_M);
    projectionMatrix.data[4 * 2 + 3] = -1.0f;

    projectionMatrix.data[4 * 3 + 2] = -FAR_CUTOFF_M * NEAR_CUTOFF_M / (FAR_CUTOFF_M - NEAR_CUTOFF_M);
    projectionMatrix.data[4 * 3 + 3] = 0;

    return projectionMatrix;
};

static const CMSISMat<4, 4> projectionMatrix = getProjectionMatrix();

static Position convertWorldFrameToCameraFrame(
    const Position &worldFrame,
    const Transform &worldToTurret)
{
    Position turretFrame = worldToTurret.apply(worldFrame);
    return turretFrame + VTM_OFFSET_FRAME;
}

/**
 * Converts a vector from world frame to screen frame.
 *
 * @param vector Vector in world frame.
 *
 * @return Result in screen frame.
 */
static ProjectedResult convertCameraFrameToScreenFrame(const Position &vector)
{
    CMSISMat<3, 1> posWorld = vector.coordinates();
    // Convert to camera axes
    CMSISMat<3, 1> posCamera = worldAxesToCameraAxes * posWorld;

    // Temporary vector since we don't have w
    CMSISMat<4, 1> vec;
    vec.data[0] = posCamera.data[0];
    vec.data[1] = posCamera.data[1];
    vec.data[2] = posCamera.data[2];
    vec.data[3] = 1.0f;

    // Conversion to screen space
    CMSISMat<4, 1> result = projectionMatrix * vec;
    result.data[0] /= result.data[3];
    result.data[1] /= result.data[3];
    result.data[2] /= result.data[3];

    ProjectedResult output;

    // Culling if out of frame
    if (abs(result.data[0]) > 1 || abs(result.data[1]) > 1 || abs(result.data[2]) > 1)
    {
        output.inFrame = false;
    }
    else
    {
        output.inFrame = true;
    }

    // Convert to screen space
    output.screenX = (result.data[0] + 1) * 0.5f * HudIndicator::SCREEN_WIDTH;
    output.screenY = (result.data[1] + 1) * 0.5f * HudIndicator::SCREEN_HEIGHT;

    return output;
}

static ProjectedResult convertWorldFrameToScreenFrame(
    const Position &worldFrame,
    const Transform &worldToTurret)
{
    Position cameraFrame = convertWorldFrameToCameraFrame(worldFrame, worldToTurret);
    return convertCameraFrameToScreenFrame(cameraFrame);
}

}  // namespace aruwsrc::control::client_display
