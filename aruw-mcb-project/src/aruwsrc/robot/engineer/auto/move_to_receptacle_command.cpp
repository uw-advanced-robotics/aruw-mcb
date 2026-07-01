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

#include "move_to_receptacle_command.hpp"

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/serial/remote.hpp"

#include "aruwsrc/control/chassis/chassis_auto_nav_controller.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/turret/algorithms/turret_controller_interface.hpp"
#include "aruwsrc/robot/engineer/algorithms/engineer_transforms.hpp"
#include "aruwsrc/robot/engineer/engineer_turret_subsystem.hpp"

namespace aruwsrc::engineer::auton
{
using tap::algorithms::WrappedFloat;
using tap::algorithms::transforms::Position;

MoveToReceptacleCommand::MoveToReceptacleCommand(
    tap::Drivers& drivers,
    aruwsrc::control::chassis::HolonomicChassisSubsystem& chassis,
    aruwsrc::control::chassis::ChassisAutoNavController& autoNavController,
    aruwsrc::engineer::EngineerTurretSubsystem& turret,
    aruwsrc::control::turret::algorithms::TurretControllerInterface& turretYawController,
    const algorithms::EngineerTransforms& transforms,
    const MoveToReceptacleConfig& config)
    : drivers_(drivers),
      chassis_(chassis),
      autoNavController_(autoNavController),
      turret_(turret),
      turretYawController_(turretYawController),
      transforms_(transforms),
      targetP_(config.pX, config.pY, 0.f),
      config_(config),
      aggregator_(
          config.xAlignToleranceM,
          config.collectWindowTowardReceptacleM,
          config.collectWindowFromReceptacleM,
          config.receptacleYPos,
          config.yTranslationFilterM,
          config.captureDelayMs)
{
    addSubsystemRequirement(&chassis);
    addSubsystemRequirement(&turret);
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void MoveToReceptacleCommand::initialize()
{
    aggregator_.reset();
    prevTurretTimeMs_ = tap::arch::clock::getTimeMilliseconds();
    turretYawController_.initialize();

    // Phase 1: line up with P.x by driving along X (hold the current Y).
    phase_ = Phase::ALIGN_X;
    startSegmentTo(Position(targetP_.x(), robotPosition().y(), 0.f));
}

void MoveToReceptacleCommand::execute()
{
    aimCameraAtReceptacle();  // every phase keeps the camera on the receptacle

    switch (phase_)
    {
        case Phase::ALIGN_X:
            driveCurrentSegment();
            if (reachedSegmentTarget())
            {
                // Phase 2: drive along Y to P, collecting poses on the way.
                phase_ = Phase::APPROACH;
                startSegmentTo(targetP_);
            }
            break;

        case Phase::APPROACH:
            driveCurrentSegment();
            collectPose();
            if (reachedSegmentTarget())
            {
                // Phase 3: sit at P and keep collecting until we have enough poses.
                phase_ = Phase::DWELL;
            }
            break;

        case Phase::DWELL:
            holdStill();
            collectPose();
            if (haveEnoughPoses())
            {
                phase_ = Phase::DONE;
            }
            break;

        case Phase::DONE:
            holdStill();
            break;
    }
}

void MoveToReceptacleCommand::end(bool) { holdStill(); }

bool MoveToReceptacleCommand::isFinished() const
{
    // Finish when the routine completes, or when the operator drops the trigger switches.
    using Remote = tap::communication::serial::Remote;
    const bool switchesUp =
        drivers_.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP &&
        drivers_.remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP;
    return phase_ == Phase::DONE || !switchesUp;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

Position MoveToReceptacleCommand::robotPosition() const
{
    return transforms_.getWorldToChassis().getTranslation();
}

void MoveToReceptacleCommand::startSegmentTo(const Position& target)
{
    segmentTarget_ = target;
    path_.resetPath();
    path_.pushPoint(robotPosition());
    path_.pushPoint(target);
    autoNavController_.attachPath(&path_);
    autoNavController_.initialize();
    autoNavController_.setDesiredSpeed(config_.approachSpeedMps);
}

bool MoveToReceptacleCommand::reachedSegmentTarget() const
{
    return Position::distance(robotPosition(), segmentTarget_) < config_.positionToleranceM;
}

void MoveToReceptacleCommand::driveCurrentSegment()
{
    const float maxWheelSpeed =
        aruwsrc::control::chassis::HolonomicChassisSubsystem::getMaxWheelSpeed(
            drivers_.refSerial.getRefSerialReceivingData(),
            drivers_.refSerial.getRobotData().chassis.powerConsumptionLimit);
    // movement on, beyblade off => drives a straight line, no spin.
    autoNavController_.runController(maxWheelSpeed, true, false);
}

void MoveToReceptacleCommand::holdStill() { chassis_.setZeroRPM(); }

void MoveToReceptacleCommand::aimCameraAtReceptacle()
{
    const uint32_t now = tap::arch::clock::getTimeMilliseconds();
    const float dt = (now - prevTurretTimeMs_) / 1000.f;
    prevTurretTimeMs_ = now;

    // The setpoint must carry the controller's own WrappedFloat bounds (an Angle, [0, 2*pi]);
    // building a fresh WrappedFloat with different bounds trips WrappedFloat::assertBoundsEqual.
    WrappedFloat yawSetpoint = turretYawController_.getSetpoint();
    yawSetpoint.setWrappedValue(config_.turretWorldYawRad);
    turretYawController_.runController(dt, yawSetpoint);
}

void MoveToReceptacleCommand::collectPose()
{
    aggregator_.tryCollect(transforms_, robotPosition(), targetP_);
}

bool MoveToReceptacleCommand::haveEnoughPoses() const
{
    return static_cast<uint32_t>(aggregator_.getPoseCount()) >= config_.minPosesToFinish;
}

}  // namespace aruwsrc::engineer::auton
