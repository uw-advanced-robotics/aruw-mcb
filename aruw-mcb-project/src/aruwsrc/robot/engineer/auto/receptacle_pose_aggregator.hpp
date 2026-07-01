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

#ifndef RECEPTACLE_POSE_AGGREGATOR_HPP_
#define RECEPTACLE_POSE_AGGREGATOR_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

#include "tap/algorithms/transforms/position.hpp"
#include "tap/algorithms/transforms/transform.hpp"

namespace aruwsrc::engineer::algorithms
{
class EngineerTransforms;
}

namespace aruwsrc::engineer::auton
{
/**
 * Collects the CV pipeline's receptacle-pose estimates while the robot is in a good position to
 * see the receptacle, and averages them into one clean worldToReceptacle transform for the arm.
 *
 * It reads the already-composed pose from EngineerTransforms (getWorldToReceptacle) rather than
 * re-transforming, and uses that pose's received-timestamp
 * (getWorldToReceptacleReceivedTimeMs) both to detect new packets and to reason about pose age.
 *
 * Frame: odometry world frame, +Y toward the receptacle (see engineer_auton_constants.hpp).
 *
 * A pose is accepted only when ALL of these hold:
 *   1. The robot is in a valid collection position: X aligned with P.x, and Y inside a window
 *      around P.y (not too close to the receptacle, not too far).
 *   2. It is a brand-new packet (its received-timestamp differs from the last one processed).
 *   3. The image was captured AFTER the robot reached the valid position. The packet's image was
 *      taken ~captureDelayMs before it arrived, so we require
 *          receivedTime - captureDelayMs >= timeReachedValidPosition.
 *      (Otherwise the pose describes the robot's OLD position and is misleading.)
 *   4. The detected receptacle's Y matches where we expect it (only Y is checked: the
 *      receptacle's X and height vary, so they can't be used to filter).
 *
 * Each accepted pose is weighted by how close the robot is to the receptacle (closer = more
 * trustworthy). computeAggregatedPose() returns the weighted average.
 */
class ReceptaclePoseAggregator
{
public:
    static constexpr size_t MAX_POSES = 40;

    /**
     * @param xAlignToleranceM         Robot X must be within this of P.x to collect.
     * @param collectTowardReceptacleM Max distance the robot may be past P toward the receptacle.
     * @param collectFromReceptacleM   Max distance the robot may be before P (away from it).
     * @param receptacleYPos           The receptacle's actual world Y (m).
     * @param yFilterM                 Reject a detection whose receptacle Y is off by more than
     * this.
     * @param captureDelayMs           Latency from image capture to packet arrival (see
     *                                 EngineerCVCommunication::ENG_CV_CAPTURE_DELAY_MS).
     */
    ReceptaclePoseAggregator(
        float xAlignToleranceM,
        float collectTowardReceptacleM,
        float collectFromReceptacleM,
        float receptacleYPos,
        float yFilterM,
        uint32_t captureDelayMs);

    /**
     * Call every tick while the robot should be collecting. Also tracks when the robot entered
     * the valid position, so must be called continuously (not only when a packet arrives).
     *
     * @param transforms  Kinematic transforms (worldToReceptacle + its received-timestamp).
     * @param robotPos    Current robot world-frame position.
     * @param targetP     The move-target P this approach is driving toward.
     */
    void tryCollect(
        const algorithms::EngineerTransforms& transforms,
        const tap::algorithms::transforms::Position& robotPos,
        const tap::algorithms::transforms::Position& targetP);

    bool hasAggregatedPose() const { return count_ > 0; }
    int getPoseCount() const { return static_cast<int>(count_); }

    /** Weighted average of all collected poses (identity if none were collected). */
    tap::algorithms::transforms::Transform computeAggregatedPose() const;

    /** Clears all collected poses. Call at the start of each approach. */
    void reset();

private:
    struct WeightedPose
    {
        float tx, ty, tz;      ///< world-frame translation
        float qw, qx, qy, qz;  ///< unit quaternion of the world-frame rotation
        float weight;
    };

    const float xAlignTolerance_;
    const float collectTowardReceptacle_;
    const float collectFromReceptacle_;
    const float receptacleYPos_;
    const float yFilter_;
    const uint32_t captureDelayMs_;

    /// When the robot most recently entered the valid collection position (-1 = not in it).
    int64_t inPositionSinceMs_ = -1;
    /// received-timestamp of the last worldToReceptacle we processed (dedup key; -1 = none).
    int64_t lastProcessedRecvTime_ = -1;

    std::array<WeightedPose, MAX_POSES> poses_{};
    size_t count_ = 0;
};

}  // namespace aruwsrc::engineer::auton

#endif  // RECEPTACLE_POSE_AGGREGATOR_HPP_
