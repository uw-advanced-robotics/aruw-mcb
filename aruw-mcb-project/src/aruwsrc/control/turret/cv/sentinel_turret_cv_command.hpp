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

#ifndef SENTINEL_TURRET_CV_COMMAND_HPP_
#define SENTINEL_TURRET_CV_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/control/subsystem.hpp"

#include "../algorithms/turret_controller_interface.hpp"
#include "../constants/turret_constants.hpp"
#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"

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
 * aims the turret accordingly using a position PID controller.
 *
 * This command, unlike the `SentinelTurretCVCommand`, is not responsible for firing projectiles
 * when the auto aim system determines it should fire. Nor does this class scan the turret back and
 * forth.
 *
 * @note If the auto aim system is offline, does not have a target acquired, or has an invalid
 * target (for example, the target is too far away), then user input from the
 * `ControlOperatorInterface` is used to control the turret instead.
 */
class SentinelTurretCVCommand : public TurretCVCommandInterface
{
public:
    /// Min scanning angle for the pitch motor since the turret doesn't need to scan all the way up
    /// (in radians)
    static constexpr float PITCH_MIN_SCAN_ANGLE = modm::toRadian(-15.0f);
    static constexpr float PITCH_MAX_SCAN_ANGLE = modm::toRadian(60.0f);

    /**
     * Scanning angle tolerance away from the min/max turret angles, in radians, at which point the
     * turret will turn around and start scanning around.
     */
    static constexpr float YAW_SCAN_ANGLE_TOLERANCE_FROM_MIN_MAX = modm::toRadian(1.0f);

    /**
     * Yaw and pitch angle increments that the turret will change by each call
     * to refresh when the turret is scanning for a target, in radians.
     */
    static constexpr float SCAN_DELTA_ANGLE = modm::toRadian(0.2f);

    /**
     * The number of times refresh is called without receiving valid CV data to when
     * the command will consider the target lost and start tracking.
     */
    static constexpr int AIM_LOST_NUM_COUNTS = 500;

    static constexpr float SCAN_LOW_PASS_ALPHA = 0.007f;

    /**
     * Constructs a TurretCVCommand
     *
     * @param[in] drivers Pointer to a global drivers object.
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
    SentinelTurretCVCommand(
        aruwsrc::Drivers *drivers,
        RobotTurretSubsystem *turretSubsystem,
        algorithms::TurretYawControllerInterface *yawController,
        algorithms::TurretPitchControllerInterface *pitchController,
        aruwsrc::algorithms::OttoBallisticsSolver *ballisticsSolver,
        const uint8_t turretID);

    void initialize() override;

    bool isReady() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override;

    const char *getName() const override { return "sentinel turret CV"; }

    ///  Request a new vision target, so it can change which robot it is targeting
    void requestNewTarget();

    /// Request the desired turret setpoint to a new currently-unseen by CV "quadrant". Useful if
    /// the sentinel is getting shot at by a robot from behind and being baited by a robot in front
    /// of it.
    void changeScanningQuadrant();

    bool getTurretID() const override { return turretID; }

    /**
     * @return True if vision is active and the turret CV command has acquired the target and the
     * turret is within some tolerance of the target. This tolerance is distance based (the further
     * away the target the closer to the center of the plate the turret must be aiming)
     */
    bool isAimingWithinLaunchingTolerance() const override { return withinAimingTolerance; }

    float fff=0;

private:
    aruwsrc::Drivers *drivers;

    RobotTurretSubsystem *turretSubsystem;

    algorithms::TurretYawControllerInterface *yawController;
    algorithms::TurretPitchControllerInterface *pitchController;

    const uint8_t turretID;

    aruwsrc::algorithms::OttoBallisticsSolver *ballisticsSolver;

    uint32_t prevTime;

    /**
     * Handles scanning logic in the pitch direction
     */
    SetpointScanner pitchScanner;

    /**
     * Handles scanning logic in the yaw direction
     */
    SetpointScanner yawScanner;

    bool scanning = false;

    bool withinAimingTolerance = false;

    /**
     * A counter that is reset to 0 every time CV starts tracking a target
     * and that keeps track of the number of times `refresh` is called when
     * an aiming solution couldn't be found (either because CV had no target
     * or aiming solution was impossible)
     */
    unsigned int lostTargetCounter = AIM_LOST_NUM_COUNTS;

    inline void enterScanMode(float yawSetpoint, float pitchSetpoint)
    {
        if (yawController != nullptr)
        {
            yawSetpoint = yawController->convertControllerAngleToChassisFrame(yawSetpoint);
        }

        lostTargetCounter = AIM_LOST_NUM_COUNTS;
        scanning = true;
        yawScanner.setScanSetpoint(yawSetpoint);
        pitchScanner.setScanSetpoint(pitchSetpoint);
    }

    inline void exitScanMode()
    {
        scanning = false;
        lostTargetCounter = 0;
    }

    /**
     * Performs a single scan iteration, updating the pitch and yaw setpoints based on the pitch/yaw
     * setpoint scanners.
     *
     * @param[out] yawSetpoint The current yaw setpoint, which this function will update
     * @param[out] pitchSetpoint The current pitch setpoint, which this function will update
     */
    void performScanIteration(float &yawSetpoint, float &pitchSetpoint);
};

}  // namespace aruwsrc::control::turret::cv

#endif  // SENTINEL_TURRET_CV_COMMAND_HPP_
