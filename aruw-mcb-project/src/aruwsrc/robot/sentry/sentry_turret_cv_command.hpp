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

#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/control/turret/algorithms/turret_controller_interface.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

#include "aruwsrc/control/turret/cv/setpoint_scanner.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command_interface.hpp"

#include "aruwsrc/robot/sentry/sentry_turret_major_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_transforms.hpp"
#include "tap/algorithms/wrapped_float.hpp"

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

namespace aruwsrc::control::turret
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
    // TODO: config someplace
    static constexpr float SCAN_TURRET_MINOR_PITCH = modm::toRadian(0.0f);

    static constexpr float SCAN_GIRLBOSS_YAW = modm::toRadian(90.0f);
    static constexpr float SCAN_MALEWIFE_YAW = modm::toRadian(-90.0f);

    /**
     * Pitch angle increments that the turret will change by each call
     * to refresh when the turret is scanning for a target, in radians.
     */
    static constexpr float YAW_SCAN_DELTA_ANGLE = modm::toRadian(0.2f);  // 0.3

    /**
     * The number of times refresh is called without receiving valid CV data to when
     * the command will consider the target lost and start tracking.
     */
    static constexpr int AIM_LOST_NUM_COUNTS = 500;

    static constexpr float SCAN_LOW_PASS_ALPHA = 0.013f;

    /**
     * Time to ignore aim requests while the turret is u-turning to aim at a new quadrant.
     */
    static constexpr uint32_t TIME_TO_IGNORE_TARGETS_WHILE_TURNING_AROUND_MS = 1'000;

    /**
     * Constructor.
     *
     * @param[in] visionCoprocessor Pointer to a global visionCoprocessor object.
     * @param[in] turretSubsystem Pointer to the turret to control.
     * @param[in] yawController Pointer to a yaw controller that will be used to control the yaw
     * axis of the turret.
     * @param[in] pitchController Pointer to a pitch controller that will be used to control the
     * pitch axis of the turret.
     * @param[in] firingCommand Pointer to command to schedule when this command deems it's time to
     * shoot.
     * @param[in] ballisticsSolver A ballistics computation engine to use for computing aiming
     * solutions.
     * @param[in] turretID The vision turet ID, must be a valid 0-based index, see VisionCoprocessor
     * for more information.
     */
    SentryTurretCVCommand(
        serial::VisionCoprocessor &visionCoprocessor,
        SentryTurretMajorSubsystem &turretMajorSubsystem,
        SentryTurretMinorSubsystem &turretMinorGirlbossSubsystem,
        SentryTurretMinorSubsystem &turretMinorMalewifeSubsystem,
        aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawControllerMajor,
        aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawControllerGirlboss,  // TODO: painnn
        aruwsrc::control::turret::algorithms::TurretPitchControllerInterface &pitchControllerGirlboss,  // Do we still need a pitch controller if pitch is constant?
        aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawControllerMalewife,
        aruwsrc::control::turret::algorithms::TurretPitchControllerInterface &pitchControllerMalewife,
        aruwsrc::algorithms::OttoBallisticsSolver &girlbossBallisticsSolver,
        aruwsrc::algorithms::OttoBallisticsSolver &malewifeBallisticsSolver,
        aruwsrc::sentry::SentryTransforms& sentryTransforms); // @todo: pass in needed transforms, not

    void initialize() ;

    bool isReady() ;

    void execute() ;

    bool isFinished() const ;

    void end(bool) ;

    const char *getName() const { return "sentry turret CV"; }

    ///  Request a new vision target, so it can change which robot it is targeting
    void requestNewTarget();

    /**
     * @return True if vision is active and the turret CV command has acquired the target and the
     * turret is within some tolerance of the target. This tolerance is distance based (the further
     * away the target the closer to the center of the plate the turret must be aiming)
     */
    bool isAimingWithinLaunchingToleranceGirlboss() const { return withinAimingToleranceGirlboss; }
    bool isAimingWithinLaunchingToleranceMalewife() const { return withinAimingToleranceMalewife; }
    // for the satisfaction of the CVOnTargetGovernor
    bool isAimingWithinLaunchingTolerance(uint8_t turretID) const { return (turretID == 0) ? withinAimingToleranceGirlboss : withinAimingToleranceMalewife; }

private:
    serial::VisionCoprocessor &visionCoprocessor;

    // TODO: control turret major
    // TODO: uhh i don't think we actually ever use the subsystems themselves lol
    SentryTurretMajorSubsystem &turretMajorSubsystem;
    RobotTurretSubsystem &turretMinorGirlbossSubsystem;
    RobotTurretSubsystem &turretMinorMalewifeSubsystem;

    aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawControllerMajor;
    aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawControllerGirlboss;
    aruwsrc::control::turret::algorithms::TurretPitchControllerInterface &pitchControllerGirlboss;
    aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawControllerMalewife;
    aruwsrc::control::turret::algorithms::TurretPitchControllerInterface &pitchControllerMalewife;

    aruwsrc::algorithms::OttoBallisticsSolver &girlbossBallisticsSolver;
    aruwsrc::algorithms::OttoBallisticsSolver &malewifeBallisticsSolver;

    aruwsrc::sentry::SentryTransforms& sentryTransforms;

    uint32_t prevTime;

    /**
     * Handles scanning logic in the yaw direction
     */
    bool scanning = false;
    tap::algorithms::WrappedFloat majorScanValue = tap::algorithms::WrappedFloat(0.0f, 0.0f, M_TWOPI);

    bool withinAimingToleranceGirlboss = false;
    bool withinAimingToleranceMalewife = false;

    tap::arch::MilliTimeout girlbossIgnoreTargetTimeout;
    tap::arch::MilliTimeout malewifeIgnoreTargetTimeout;

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
    inline void enterScanMode(float majorYawSetpoint)
    {
        lostTargetCounter = AIM_LOST_NUM_COUNTS;
        scanning = true;
        // @todo set function for wrapped float that retains bounds
        majorScanValue = tap::algorithms::WrappedFloat(majorYawSetpoint, 0.0f, M_TWOPI);
    }

    inline void exitScanMode()
    {
        scanning = false;
        lostTargetCounter = 0;
    }
};

}  // namespace aruwsrc::control::turret::cv

#endif  // SENTRY_TURRET_CV_COMMAND_HPP_
