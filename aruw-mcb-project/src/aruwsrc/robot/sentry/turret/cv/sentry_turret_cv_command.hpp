/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTRY_TURRET_CV_COMMAND_HPP_
#define SENTRY_TURRET_CV_COMMAND_HPP_

#include <aruwsrc/algorithms/plate_hit_tracker.hpp>

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/algorithms/cv_ballistics_solver.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/turret/algorithms/turret_controller_interface.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/cv/setpoint_scanner.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command_interface.hpp"
#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"
#include "aruwsrc/robot/sentry/algorithms/odometry/sentry_transforms.hpp"
#include "aruwsrc/robot/sentry/turret/sentry_turret_minor_subsystem.hpp"

namespace tap::control::odometry
{
class Odometry2DInterface;
}

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::turret
{
class RobotTurretSubsystem;
}

namespace aruwsrc::control::launcher
{
class LaunchSpeedPredictorInterface;
}

namespace aruwsrc::sentry::turret::cv
{
/**
 * A command that receives input from the vision system via the `VisionCoprocessor` driver and
 * aims the turrets accordingly using a position PID controller.
 *
 * Coordinates turret major and minors to scan/target while maintaining FOV and view of direction
 * of movement. (This is why we need both minors controlled by a single command.)
 */
class SentryTurretCVCommand : public tap::control::Command
{
public:
    struct TurretConfig
    {
        SentryTurretMinorSubsystem &turretSubsystem;
        control::turret::algorithms::TurretAxisControllerInterface<
            control::turret::algorithms::Axis::YAW> &yawController;
        control::turret::algorithms::TurretAxisControllerInterface<
            control::turret::algorithms::Axis::PITCH> &pitchController;
        aruwsrc::algorithms::CvBallisticsSolver &ballisticsSolver;
    };

    enum HitState
    {
        HIT,
        NOT_HIT,
    };

    static constexpr float YAW_SCAN_DELTA_ANGLE = modm::toRadian(0.60f);

    /**
     * The number of times refresh is called without receiving valid CV data to when
     * the command will consider the target lost and start tracking.
     */
    static constexpr int AIM_LOST_NUM_COUNTS = 500;

    /**
     * Time to ignore aim requests while the turret is u-turning to aim at a new quadrant.
     */
    static constexpr uint32_t TIME_TO_IGNORE_TARGETS_WHILE_TURNING_AROUND_MS = 1'000;

    /**
     * Constructor.
     *
     * @param[in] visionCoprocessor Pointer to a global visionCoprocessor object.
     * # TODO: docstring
     */
    SentryTurretCVCommand(
        communication::serial::VisionCoprocessor &visionCoprocessor,
        aruwsrc::algorithms::PlateHitTracker &plateHitTracker,
        aruwsrc::control::turret::YawTurretSubsystem &turretMajorSubsystem,
        aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
            aruwsrc::control::turret::algorithms::Axis::YAW> &yawControllerMajor,
        TurretConfig &turretWidowConfig,
        aruwsrc::sentry::algorithms::odometry::SentryTransforms &sentryTransforms);

    void initialize();

    bool isReady();

    void execute();

    bool isFinished() const;

    void end(bool);

    const char *getName() const { return "sentry turret CV command"; }

    ///  Request a new vision target, so it can change which robot it is targeting
    void requestNewTarget();

    /**
     * @return True if vision is active and the turret CV command has acquired the target and the
     * turret is within some tolerance of the target. This tolerance is distance based (the further
     * away the target the closer to the center of the plate the turret must be aiming)
     */
    bool isAimingWithinLaunchingTolerance(uint8_t turretID) const
    {
        if (turretID != turretWidowConfig.turretSubsystem.getTurretID())
        {
            return false;
        }
        return withinAimingToleranceWidow;
    }

private:
    /**
     * Converts the angles contained in the ballistics solution to the frame of the turret major,
     * since chassis-frame controllers are used
     */
    void computeAimSetpoints(
        TurretConfig &config,
        aruwsrc::algorithms::CvBallisticsSolver::BallisticsSolution &solution,
        WrappedFloat *desiredYawSetpoint,
        WrappedFloat *desiredPitchSetpoint,
        bool *withinAimingTolerance);

    communication::serial::VisionCoprocessor &visionCoprocessor;
    aruwsrc::algorithms::PlateHitTracker &plateHitTracker;

    aruwsrc::control::turret::YawTurretSubsystem &turretMajorSubsystem;
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        aruwsrc::control::turret::algorithms::Axis::YAW> &yawControllerMajor;

    TurretConfig &turretWidowConfig;
    aruwsrc::sentry::algorithms::odometry::SentryTransforms &sentryTransforms;

    uint32_t prevTime;

    /**
     * Handles scanning logic in the yaw direction
     */
    bool scanning = false;
    bool targetFound = false;

    HitState curHitState = HitState::NOT_HIT;
    HitState lastHitState = HitState::NOT_HIT;
    uint32_t lastHitTime = 0;
    aruwsrc::algorithms::PlateHitTracker::PlateHitBinData plateHitData{};
    aruwsrc::algorithms::PlateHitTracker::PlateHitBinData lastPlateHitData{};
    float hitLocDiffRads = 0.0f;

    static constexpr uint32_t HIT_COUNT_DELAY_MILLISEC = 500;
    static constexpr float HIT_MAG_THRESH = 0.4f;
    static constexpr float HIT_DIFF_OFFSET = modm::toRadian(20.0f);

    static constexpr float SCAN_LOW_PASS_ALPHA = 0.035f;
    static constexpr float SCAN_ENDPOINT_TOLERANCE = modm::toRadian(1.0f);

    static constexpr int SCAN_CLOCKWISE = -1;
    static constexpr int SCAN_COUNTER_CLOCKWISE = 1;
    int scanDir = SCAN_COUNTER_CLOCKWISE;
    int pitchScanDir = SCAN_CLOCKWISE;

    // Scan two full rotations per pass before reversing, with a small overscan to avoid wrapped
    // endpoint ambiguity.
    static constexpr float YAW_SCAN_HALF_RANGE = M_TWOPI + modm::toRadian(4.0f);
    static constexpr float MAJOR_SCAN_HALF_RANGE = YAW_SCAN_HALF_RANGE;
    static constexpr float MAJOR_SCAN_RATIO = MAJOR_SCAN_HALF_RANGE / YAW_SCAN_HALF_RANGE;
    static constexpr float SCAN_TURRET_MINOR_UP_PITCH =
        aruwsrc::control::turret::turretWidow::PITCH_MOTOR_CONFIG.minAngle;
    static constexpr float SCAN_TURRET_MINOR_DOWN_PITCH = modm::toRadian(25.0f);
    static constexpr float PITCH_SCAN_DELTA_ANGLE = modm::toRadian(0.19f);

    tap::algorithms::WrappedFloat scanCenter = Angle(0);
    float scanOffsetFromCenter = 0.0f;
    float pitchScanValue = SCAN_TURRET_MINOR_UP_PITCH;
    tap::algorithms::WrappedFloat minorScanValue = Angle(0);
    tap::algorithms::WrappedFloat majorScanValue = Angle(0);

    bool withinAimingToleranceWidow = false;

    /**
     * A counter that is reset to 0 every time CV starts tracking a target
     * and that keeps track of the number of times `refresh` is called when
     * an aiming solution couldn't be found (either because CV had no target
     * or aiming solution was impossible)
     */
    unsigned int lostTargetCounter = AIM_LOST_NUM_COUNTS;

    /**
     * Initializes scanning mode.
     *
     * Centers the scan on the turret major and starts the smooth yaw sweep from the current minor
     * yaw setpoint.
     */
    inline void enterScanMode(WrappedFloat majorYawSetpoint, WrappedFloat minorYawSetpoint)
    {
        lostTargetCounter = AIM_LOST_NUM_COUNTS;
        scanning = true;
        scanCenter = majorYawSetpoint;
        scanOffsetFromCenter = tap::algorithms::limitVal(
            majorYawSetpoint.minDifference(minorYawSetpoint),
            -YAW_SCAN_HALF_RANGE,
            YAW_SCAN_HALF_RANGE);
        minorScanValue = scanCenter + scanOffsetFromCenter;
        majorScanValue = majorYawSetpoint;
        scanDir = (scanOffsetFromCenter >= 0.0f) ? SCAN_COUNTER_CLOCKWISE : SCAN_CLOCKWISE;
        pitchScanValue = SCAN_TURRET_MINOR_UP_PITCH;
        pitchScanDir = SCAN_COUNTER_CLOCKWISE;
    }

    inline void exitScanMode()
    {
        scanning = false;
        lostTargetCounter = 0;
    }
};

}  // namespace aruwsrc::sentry::turret::cv

#endif  // SENTRY_TURRET_CV_COMMAND_HPP_
