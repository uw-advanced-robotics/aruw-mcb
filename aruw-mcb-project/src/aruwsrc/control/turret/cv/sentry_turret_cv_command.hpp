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

#include "../algorithms/turret_controller_interface.hpp"
#include "../constants/turret_constants.hpp"
#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

#include "setpoint_scanner.hpp"
#include "turret_cv_command_interface.hpp"

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

namespace aruwsrc::control::turret::cv
{
/**
 * A command that receives input from the vision system via the `VisionCoprocessor` driver and
 * aims the turrets accordingly using a position PID controller.
 * 
 * Coordinates turret major and minors to scan/target while maintaining FOV and view of direction
 * of movement. (This is why we need both minors controlled by a single command.)
 */
class SentryTurretCVCommand : public TurretCVCommandInterface
{
public:
    // TODO: config someplace
    static constexpr float MINOR_TURRET_PITCH = modm::toRadian(0.0f);

    static constexpr float YAW_GIRLBOSS_MIN = modm::toRadian(10.0f);
    static constexpr float YAW_GIRLBOSS_MAX = modm::toRadian(10.0f);
    static constexpr float YAW_MALEWIFE_MIN = modm::toRadian(10.0f);
    static constexpr float YAW_MALEWIFE_MIN = modm::toRadian(10.0f);
    /**
     * Scanning angle tolerance away from the min/max turret angles, in radians, at which point the
     * turret will turn around and start scanning around.
     */
    static constexpr float YAW_SCAN_ANGLE_TOLERANCE_FROM_MIN_MAX = modm::toRadian(0.5f);

    /**
     * Pitch angle increments that the turret will change by each call
     * to refresh when the turret is scanning for a target, in radians.
     */
    static constexpr float PITCH_SCAN_DELTA_ANGLE = modm::toRadian(0.4f);

    /**
     * Yaw angle increments that the turret will change by each call
     * to refresh when the turret is scanning for a target, in radians.
     */
    static constexpr float YAW_SCAN_DELTA_ANGLE = modm::toRadian(0.3f);

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
     * Constructs a TurretCVCommand
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
        RobotTurretSubsystem &turretMajorSubsystem,
        RobotTurretSubsystem &turretMinorGirlbossSubsystem,
        RobotTurretSubsystem &turretMinorMalewifeSubsystem,
        algorithms::TurretYawControllerInterface &yawControllerGirlboss,  // TODO: painnn
        algorithms::TurretPitchControllerInterface &pitchControllerGirlboss,  // Do we still need a pitch controller if pitch is constant?
        algorithms::TurretYawControllerInterface &yawControllerMalewife,
        algorithms::TurretPitchControllerInterface &pitchControllerMalewife,
        aruwsrc::algorithms::OttoBallisticsSolver &girlbossBallisticsSolver,
        aruwsrc::algorithms::OttoBallisticsSolver &malewifeBallisticsSolver);

    void initialize() override;

    bool isReady() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override;

    const char *getName() const override { return "sentry turret CV"; }

    ///  Request a new vision target, so it can change which robot it is targeting
    void requestNewTarget();

    /**
     * @return True if vision is active and the turret CV command has acquired the target and the
     * turret is within some tolerance of the target. This tolerance is distance based (the further
     * away the target the closer to the center of the plate the turret must be aiming)
     */
    bool isAimingWithinLaunchingTolerance() const override { return withinAimingTolerance; }

private:
    serial::VisionCoprocessor &visionCoprocessor;

    RobotTurretSubsystem &turretMajorSubsystem;
    RobotTurretSubsystem &turretMinorGirlbossSubsystem;
    RobotTurretSubsystem &turretMinorMalewifeSubsystem;

    algorithms::TurretYawControllerInterface &yawControllerGirlboss;
    algorithms::TurretPitchControllerInterface &pitchControllerGirlboss;
    algorithms::TurretYawControllerInterface &yawControllerMalewife;
    algorithms::TurretPitchControllerInterface &pitchControllerMalewife;

    aruwsrc::algorithms::OttoBallisticsSolver &girlbossBallisticsSolver;
    aruwsrc::algorithms::OttoBallisticsSolver &malewifeBallisticsSolver;

    uint32_t prevTime;

    /**
     * Handles scanning logic in the yaw direction
     */
    SetpointScanner yawGirlbossScanner;
    SetpointScanner yawMalewifeScanner;

    bool scanning = false;

    bool withinAimingTolerance = false;

    tap::arch::MilliTimeout ignoreTargetTimeout;

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
     * Sets the yaw scanners to the current setpoints of the turret minor controllers.
     * @param girlbossYawSetpoint The current setpoint returned from the girlboss yaw controller.
     * @param malewifeYawSetpoint The current setpoint returned from the malewife yaw controller.
    */
    inline void enterScanMode(float girlbossYawSetpoint, float malewifeYawSetpoint)
    {
        // FIXME: coordinate yawSetpoints when entering scan
        float yawSetpointGirlboss = yawControllerGirlboss.convertControllerAngleToChassisFrame(girlbossYawSetpoint);
        float yawSetpointMalewife = yawControllerMalewife.convertControllerAngleToChassisFrame(malewifeYawSetpoint);

        lostTargetCounter = AIM_LOST_NUM_COUNTS;
        scanning = true;
        yawGirlbossScanner.setScanSetpoint(yawSetpointGirlboss);
        yawMalewifeScanner.setScanSetpoint(yawSetpointMalewife);
    }

    inline void exitScanMode()
    {
        scanning = false;
        lostTargetCounter = 0;
    }

    /**
     * Performs a single scan iteration, updating the yaw setpoint based on the yaw
     * setpoint scanners.
     *
     * @param[out] yawSetpoint The current yaw setpoint, which this function will update
     */
    void performScanIteration(float &yawSetpoint);
};

}  // namespace aruwsrc::control::turret::cv

#endif  // SENTRY_TURRET_CV_COMMAND_HPP_
