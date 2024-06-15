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

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/control/command.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/turret/algorithms/turret_controller_interface.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/cv/setpoint_scanner.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command_interface.hpp"
#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_ballistics_solver.hpp"
#include "aruwsrc/robot/sentry/sentry_transforms.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"

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

namespace aruwsrc::control::sentry
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
        aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawController;
        aruwsrc::control::turret::algorithms::TurretPitchControllerInterface &pitchController;
        aruwsrc::sentry::SentryBallisticsSolver &ballisticsSolver;
    };

    static constexpr float SCAN_TURRET_MINOR_PITCH = modm::toRadian(10.0f);

    static constexpr float SCAN_TURRET_LEFT_YAW = modm::toRadian(90.0f);
    static constexpr float SCAN_TURRET_RIGHT_YAW = modm::toRadian(-90.0f);

    /**
     * Pitch angle increments that the turret will change by each call
     * to refresh when the turret is scanning for a target, in radians.
     */
    static constexpr float YAW_SCAN_DELTA_ANGLE = modm::toRadian(0.12f);

    /**
     * The number of times refresh is called without receiving valid CV data to when
     * the command will consider the target lost and start tracking.
     */
    static constexpr int AIM_LOST_NUM_COUNTS = 500;

    static constexpr float SCAN_LOW_PASS_ALPHA = 0.007f;

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
        serial::VisionCoprocessor &visionCoprocessor,
        tap::communication::serial::RefSerial &refSerial,
        aruwsrc::control::turret::YawTurretSubsystem &turretMajorSubsystem,
        aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawControllerMajor,
        TurretConfig &turretLeftConfig,
        TurretConfig &turretRightConfig,
        aruwsrc::sentry::SentryTransforms &sentryTransforms);

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
        return turretID == turretLeftConfig.turretSubsystem.getTurretID()
                   ? withinAimingToleranceLeft
                   : withinAimingToleranceRight;
    }

private:
    /**
     * Converts the angles contained in the ballistics solution to the frame of the turret major,
     * since chassis-frame controllers are used
     */
    void computeAimSetpoints(
        TurretConfig &config,
        aruwsrc::sentry::SentryBallisticsSolver::BallisticsSolution &solution,
        WrappedFloat *desiredYawSetpoint,
        WrappedFloat *desiredPitchSetpoint,
        bool *withinAimingTolerance);

    serial::VisionCoprocessor &visionCoprocessor;
    tap::communication::serial::RefSerial &refSerial;

    aruwsrc::control::turret::YawTurretSubsystem &turretMajorSubsystem;
    aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawControllerMajor;

    TurretConfig &turretLeftConfig;
    TurretConfig &turretRightConfig;
    aruwsrc::sentry::SentryTransforms &sentryTransforms;

    uint32_t prevTime;

    /**
     * Handles scanning logic in the yaw direction
     */
    bool scanning = false;
    bool targetFound = false;

    // scan direction
    static constexpr int SCAN_CLOCKWISE = -1;
    static constexpr int SCAN_COUNTER_CLOCKWISE = 1;
    int scanDir = 1;

    // scan between 90 and 270 to avoid any silliness from wrapping
    static constexpr float CW_TO_CCW_WRAP_VALUE = modm::toRadian(45.0f);
    static constexpr float CCW_TO_CW_WRAP_VALUE = modm::toRadian(315.0f);

    tap::algorithms::WrappedFloat majorScanValue = Angle(0);

    bool withinAimingToleranceLeft = false;
    bool withinAimingToleranceRight = false;

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
     * Sets the yaw scanner to the current setpoint of the turret major.
     */
    inline void enterScanMode(WrappedFloat majorYawSetpoint)
    {
        lostTargetCounter = AIM_LOST_NUM_COUNTS;
        scanning = true;
        majorScanValue = majorYawSetpoint;
    }

    inline void exitScanMode()
    {
        scanning = false;
        lostTargetCounter = 0;
    }
};

}  // namespace aruwsrc::control::sentry

#endif  // SENTRY_TURRET_CV_COMMAND_HPP_
