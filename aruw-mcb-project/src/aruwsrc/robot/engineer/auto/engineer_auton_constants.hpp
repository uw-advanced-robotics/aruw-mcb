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

#ifndef ENGINEER_AUTON_CONSTANTS_HPP_
#define ENGINEER_AUTON_CONSTANTS_HPP_

#include <cmath>

#include "aruwsrc/communication/serial/engineer_cv_communication.hpp"

#include "move_to_receptacle_command.hpp"

namespace aruwsrc::engineer::auton
{
// ===========================================================================
// COORDINATE FRAME (odometry world frame)
//   +X = the direction the robot faced at IMU calibration ("up the field")
//   +Y = the robot's left at calibration (TOWARD the receptacles)
//   +Z = up
// So the robot drives in +Y to approach a receptacle, and the camera faces it
// when the turret yaw = +pi/2. Always calibrate facing up the field.
//
// EACH RECEPTACLE gets one MoveToReceptacleConfig below (numbers inline). What
// every field means (see also the struct in move_to_receptacle_command.hpp):
//
//   pX, pY                         the move-target P: where we drive the chassis. Roughly
//                                  (receptacle X, receptacle Y - camera standoff), so the
//                                  camera sees the whole receptacle when parked at P.
//   approachSpeedMps               drive speed (m/s). 0.075 = 5% of a 1.5 m/s full speed;
//                                  raise cautiously.
//   positionToleranceM             a straight segment counts as reached within this (m).
//   xAlignToleranceM               collect poses only when robot X is this close to P.x (m).
//   collectWindowTowardReceptacleM collect up to this far PAST P toward the receptacle (m);
//                                  beyond it the receptacle overfills the camera frame.
//   collectWindowFromReceptacleM   collect up to this far BEFORE P away from it (m); beyond it
//                                  detection is unreliable.
//   receptacleYPos                 the receptacle's ACTUAL world Y (m). Only Y is known (X and
//                                  height vary). Used for the Y filter + distance weighting.
//   yTranslationFilterM            reject a detection whose receptacle Y is off by more than this.
//   captureDelayMs                 image-capture-to-packet latency (ms); a pose is trusted only
//                                  if its image was taken after the robot reached the position.
//   minPosesToFinish               sit at P collecting until at least this many poses, then done.
//   turretWorldYawRad              world yaw to hold the camera at (+pi/2 = +Y, the receptacles).
// ===========================================================================

/// EASY receptacle — the one we currently run.
static constexpr MoveToReceptacleConfig EASY_RECEPTACLE_CONFIG = {
    .pX = 0.3f,
    .pY = 0.2f,
    .approachSpeedMps = 0.075f,
    .positionToleranceM = 0.05f,
    .xAlignToleranceM = 0.05f,
    .collectWindowTowardReceptacleM = 0.10f,
    .collectWindowFromReceptacleM = 2.0f,
    .receptacleYPos = 0.8f,
    .yTranslationFilterM = 0.4f,
    .captureDelayMs = communication::serial::EngineerCVCommunication::ENG_CV_CAPTURE_DELAY_MS,
    .minPosesToFinish = 5,
    .turretWorldYawRad = M_PI_2,
};

// /// MEDIUM receptacle — fill in at competition, then uncomment and add a command for it.
// static constexpr MoveToReceptacleConfig MEDIUM_RECEPTACLE_CONFIG = {
//     .pX = ,
//     .pY = ,
//     .approachSpeedMps = ,
//     .positionToleranceM = ,
//     .xAlignToleranceM = ,
//     .collectWindowTowardReceptacleM = ,
//     .collectWindowFromReceptacleM = ,
//     .receptacleYPos = ,
//     .yTranslationFilterM = ,
//     .captureDelayMs = communication::serial::EngineerCVCommunication::ENG_CV_CAPTURE_DELAY_MS,
//     .minPosesToFinish = ,
//     .turretWorldYawRad = M_PI_2,
// };

// /// HARD receptacle — fill in at competition, then uncomment and add a command for it.
// static constexpr MoveToReceptacleConfig HARD_RECEPTACLE_CONFIG = {
//     .pX = ,
//     .pY = ,
//     .approachSpeedMps = ,
//     .positionToleranceM = ,
//     .xAlignToleranceM = ,
//     .collectWindowTowardReceptacleM = ,
//     .collectWindowFromReceptacleM = ,
//     .receptacleYPos = ,
//     .yTranslationFilterM = ,
//     .captureDelayMs = communication::serial::EngineerCVCommunication::ENG_CV_CAPTURE_DELAY_MS,
//     .minPosesToFinish = ,
//     .turretWorldYawRad = M_PI_2,
// };

}  // namespace aruwsrc::engineer::auton

#endif  // ENGINEER_AUTON_CONSTANTS_HPP_
