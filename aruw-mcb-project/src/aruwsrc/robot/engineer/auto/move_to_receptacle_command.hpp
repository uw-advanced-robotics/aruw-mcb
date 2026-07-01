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

#ifndef MOVE_TO_RECEPTACLE_COMMAND_HPP_
#define MOVE_TO_RECEPTACLE_COMMAND_HPP_

#include <cstdint>

#include "tap/algorithms/transforms/position.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/algorithms/auto_nav_path.hpp"

#include "receptacle_pose_aggregator.hpp"

namespace aruwsrc::control::chassis
{
class HolonomicChassisSubsystem;
class ChassisAutoNavController;
}  // namespace aruwsrc::control::chassis
namespace aruwsrc::control::turret::algorithms
{
class TurretControllerInterface;
}
namespace aruwsrc::engineer
{
class EngineerTurretSubsystem;
}
namespace aruwsrc::engineer::algorithms
{
class EngineerTransforms;
}

namespace aruwsrc::engineer::auton
{
/**
 * A full per-receptacle spec for MoveToReceptacleCommand (move-target P + all tuning). The same
 * command works for any receptacle by swapping the config. Fields are documented inline below and
 * in engineer_auton_constants.hpp, where the three receptacles' configs are defined.
 */
struct MoveToReceptacleConfig
{
    float pX;                  ///< move-target P, world X (m).
    float pY;                  ///< move-target P, world Y (m).
    float approachSpeedMps;    ///< chassis translation speed during the drive (m/s).
    float positionToleranceM;  ///< a straight segment counts as reached within this (m).
    float xAlignToleranceM;    ///< collect poses only when robot X within this of P.x (m).
    float collectWindowTowardReceptacleM;  ///< collect up to this far PAST P toward the receptacle
                                           ///< (m).
    float collectWindowFromReceptacleM;    ///< collect up to this far BEFORE P away from it (m).
    float receptacleYPos;                  ///< the receptacle's actual world Y (m); used for the Y
                                           ///< filter and distance weighting.
    float yTranslationFilterM;  ///< reject a detection whose receptacle Y is off by this (m).
    uint32_t captureDelayMs;    ///< image-capture-to-packet latency; a pose is trusted
                                ///< only if its image was taken after reaching the
                                ///< collect position.
    uint32_t minPosesToFinish;  ///< sit at P collecting until at least this many poses.
    float turretWorldYawRad;    ///< world yaw to hold the camera at (+pi/2 = +Y).
};

/**
 * Drives the chassis up to the move-target P and collects CV receptacle-pose estimates, then
 * finishes. The averaged pose is read afterwards (getAggregatedReceptaclePose()) by a follow-on
 * command (e.g. arm IK). See engineer_auton_constants.hpp for the coordinate frame.
 *
 * The motion is a simple phase machine; each drive phase drives one straight, axis-aligned
 * segment via the shared ChassisAutoNavController:
 *   ALIGN_X  - drive along X to line up with P.x.
 *   APPROACH - drive along Y to P, collecting poses along the way.
 *   DWELL    - sit at P (still) and keep collecting until minPosesToFinish poses are gathered.
 *   DONE     - finished.
 *
 * Throughout, the turret yaw holds a fixed world angle so the (turret-mounted) camera faces the
 * receptacle. Turret pitch is left alone because it does not move the camera.
 */
class MoveToReceptacleCommand : public tap::control::Command
{
public:
    MoveToReceptacleCommand(
        tap::Drivers& drivers,
        aruwsrc::control::chassis::HolonomicChassisSubsystem& chassis,
        aruwsrc::control::chassis::ChassisAutoNavController& autoNavController,
        aruwsrc::engineer::EngineerTurretSubsystem& turret,
        aruwsrc::control::turret::algorithms::TurretControllerInterface& turretYawController,
        const algorithms::EngineerTransforms& transforms,
        const MoveToReceptacleConfig& config);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

    const char* getName() const override { return "move to receptacle"; }

    /** Retarget a different move-point (takes effect on the next initialize()). */
    void setTarget(const tap::algorithms::transforms::Position& targetP) { targetP_ = targetP; }

    // --- results, read by the follow-on (e.g. IK) command after this one finishes ---
    bool hasAggregatedPose() const { return aggregator_.hasAggregatedPose(); }
    int getPoseCount() const { return aggregator_.getPoseCount(); }
    tap::algorithms::transforms::Transform getAggregatedReceptaclePose() const
    {
        return aggregator_.computeAggregatedPose();
    }

private:
    enum class Phase
    {
        ALIGN_X,
        APPROACH,
        DWELL,
        DONE,
    };

    tap::Drivers& drivers_;
    aruwsrc::control::chassis::HolonomicChassisSubsystem& chassis_;
    aruwsrc::control::chassis::ChassisAutoNavController& autoNavController_;
    aruwsrc::engineer::EngineerTurretSubsystem& turret_;
    aruwsrc::control::turret::algorithms::TurretControllerInterface& turretYawController_;
    const algorithms::EngineerTransforms& transforms_;

    tap::algorithms::transforms::Position targetP_;
    const MoveToReceptacleConfig config_;

    aruwsrc::algorithms::AutoNavPath path_;
    ReceptaclePoseAggregator aggregator_;

    Phase phase_ = Phase::DONE;
    tap::algorithms::transforms::Position segmentTarget_{0.f, 0.f, 0.f};
    uint32_t prevTurretTimeMs_ = 0;

    // --- small helpers, named so the phase logic reads like prose ---
    tap::algorithms::transforms::Position robotPosition() const;
    void startSegmentTo(const tap::algorithms::transforms::Position& target);
    bool reachedSegmentTarget() const;
    void driveCurrentSegment();
    void holdStill();
    void aimCameraAtReceptacle();
    void collectPose();
    bool haveEnoughPoses() const;
};

}  // namespace aruwsrc::engineer::auton

#endif  // MOVE_TO_RECEPTACLE_COMMAND_HPP_
