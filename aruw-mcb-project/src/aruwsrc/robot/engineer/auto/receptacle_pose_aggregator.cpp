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

#include "receptacle_pose_aggregator.hpp"

#include <cmath>

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/architecture/clock.hpp"

#include "aruwsrc/robot/engineer/algorithms/engineer_transforms.hpp"
#include "modm/math/geometry/quaternion.hpp"
#include "modm/math/geometry/vector3.hpp"
#include "modm/math/matrix.hpp"

namespace aruwsrc::engineer::auton
{
using tap::algorithms::CMSISMat;
using tap::algorithms::transforms::Orientation;
using tap::algorithms::transforms::Position;
using tap::algorithms::transforms::Transform;
using Quat = modm::Quaternion<float>;
using V3 = modm::Vector<float, 3>;

ReceptaclePoseAggregator::ReceptaclePoseAggregator(
    float xAlignToleranceM,
    float collectTowardReceptacleM,
    float collectFromReceptacleM,
    float receptacleYPos,
    float yFilterM,
    uint32_t captureDelayMs)
    : xAlignTolerance_(xAlignToleranceM),
      collectTowardReceptacle_(collectTowardReceptacleM),
      collectFromReceptacle_(collectFromReceptacleM),
      receptacleYPos_(receptacleYPos),
      yFilter_(yFilterM),
      captureDelayMs_(captureDelayMs)
{
}

void ReceptaclePoseAggregator::reset()
{
    count_ = 0;
    inPositionSinceMs_ = -1;
    lastProcessedRecvTime_ = -1;
}

/**
 * Builds the unit quaternion for an XYZ-Euler rotation (roll about X, pitch about Y, yaw about
 * Z), matching tap::transforms::Orientation. Uses modm's axis-angle Quaternion constructor and
 * quaternion multiply rather than hand-rolled trigonometry.
 */
static Quat eulerToQuat(float roll, float pitch, float yaw)
{
    Quat qx(V3(1.f, 0.f, 0.f), roll);
    Quat qy(V3(0.f, 1.f, 0.f), pitch);
    Quat qz(V3(0.f, 0.f, 1.f), yaw);
    Quat q = qz * qy * qx;
    q.normalize();
    return q;
}

void ReceptaclePoseAggregator::tryCollect(
    const algorithms::EngineerTransforms& transforms,
    const Position& robotPos,
    const Position& targetP)
{
    if (count_ >= MAX_POSES) return;

    const int64_t now = tap::arch::clock::getTimeMilliseconds();

    // (1) Is the robot in a valid collection position? X aligned with P.x, and Y inside the
    //     window around P (+Y is toward the receptacle). Track WHEN it entered this position.
    const bool xAligned = fabsf(robotPos.x() - targetP.x()) <= xAlignTolerance_;
    const float yPastP = robotPos.y() - targetP.y();  // >0: past P toward the receptacle
    const bool yInWindow = yPastP <= collectTowardReceptacle_ && yPastP >= -collectFromReceptacle_;
    if (!xAligned || !yInWindow)
    {
        inPositionSinceMs_ = -1;  // out of position; restart the entry clock next time
        return;
    }
    if (inPositionSinceMs_ == -1) inPositionSinceMs_ = now;  // just entered the valid position

    // (2) Only act on a brand-new packet (its received-timestamp differs from the last one).
    const int64_t recvTime = transforms.getWorldToReceptacleReceivedTimeMs();
    if (recvTime == -1) return;                      // no valid pose has ever arrived
    if (recvTime == lastProcessedRecvTime_) return;  // already handled this packet
    lastProcessedRecvTime_ = recvTime;

    // (3) The image was taken ~captureDelayMs before the packet arrived. Require it to have been
    //     captured AFTER the robot reached the valid position, i.e.
    //         recvTime - captureDelayMs >= inPositionSinceMs
    if (recvTime - static_cast<int64_t>(captureDelayMs_) < inPositionSinceMs_) return;

    // (4) Reject if the detected receptacle Y is not where we expect it. Only Y is known
    //     (the receptacle's X and height vary), so only Y can be filtered.
    const Transform& worldToReceptacle = transforms.getWorldToReceptacle();
    if (fabsf(worldToReceptacle.getY() - receptacleYPos_) > yFilter_) return;

    // Store, weighted by closeness to the receptacle (closer => more trustworthy).
    const float distToReceptacle = receptacleYPos_ - robotPos.y();
    const float weight = 1.f / fmaxf(distToReceptacle, 0.05f);
    const Quat q = eulerToQuat(
        worldToReceptacle.getRoll(),
        worldToReceptacle.getPitch(),
        worldToReceptacle.getYaw());

    poses_[count_] = {
        worldToReceptacle.getX(),
        worldToReceptacle.getY(),
        worldToReceptacle.getZ(),
        q.w,
        q.x,
        q.y,
        q.z,
        weight};
    count_++;
}

Transform ReceptaclePoseAggregator::computeAggregatedPose() const
{
    if (count_ == 0) return Transform::identity();

    // Translation: weighted mean. Rotation: weighted average of unit quaternions.
    float sumW = 0.f, sumX = 0.f, sumY = 0.f, sumZ = 0.f;
    Quat acc(0.f, 0.f, 0.f, 0.f);
    const Quat ref(poses_[0].qw, poses_[0].qx, poses_[0].qy, poses_[0].qz);

    for (size_t i = 0; i < count_; i++)
    {
        const WeightedPose& p = poses_[i];
        sumW += p.weight;
        sumX += p.weight * p.tx;
        sumY += p.weight * p.ty;
        sumZ += p.weight * p.tz;

        Quat q(p.qw, p.qx, p.qy, p.qz);
        // A quaternion and its negative are the same rotation; flip into a common hemisphere so
        // the linear average is meaningful.
        const float dot = q.w * ref.w + q.x * ref.x + q.y * ref.y + q.z * ref.z;
        if (dot < 0.f) q = q * -1.f;
        acc = acc + (q * p.weight);
    }

    acc.normalize();

    const Position translation(sumX / sumW, sumY / sumW, sumZ / sumW);

    // Quaternion -> rotation matrix. modm stores it column-major (element[col*3 + row]);
    // CMSISMat is row-major (data[row*3 + col]), so this copy transposes the storage layout.
    modm::Matrix<float, 3, 3> m;
    acc.to3x3Matrix(&m);
    float rowMajor[9];
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            rowMajor[row * 3 + col] = m.element[col * 3 + row];
        }
    }

    return Transform(translation, Orientation(CMSISMat<3, 3>(rowMajor)));
}

}  // namespace aruwsrc::engineer::auton
