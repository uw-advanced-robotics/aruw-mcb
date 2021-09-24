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

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/comprised_command.hpp"
#include "tap/control/turret/turret_subsystem_interface.hpp"

#include "aruwsrc/control/sentinel/firing/sentinel_rotate_agitator_command.hpp"
#include "aruwsrc/control/sentinel/firing/sentinel_switcher_subsystem.hpp"

namespace aruwsrc::agitator
{
class AgitatorSubsystem;
}

namespace aruwsrc::control::turret
{
/**
 * A command that receives input from the vision system via the `XavierSerial` driver and aims the
 * turret accordingly.
 */
class SentinelTurretCVCommand : public tap::control::ComprisedCommand
{
public:
    /**
     * Pitch/yaw error margins within which the auto aim deems it acceptable
     * to fire the launcher, in degrees.
     */
    static constexpr float YAW_FIRE_ERROR_MARGIN = 2.0f;
    static constexpr float PITCH_FIRE_ERROR_MARGIN = 2.0f;

    /**
     * Pitch/yaw angle increments that the turret will change by each call
     * to refresh when the turret is scanning for a target, in degrees.
     */
    static constexpr float SCAN_DELTA_ANGLE_YAW = 0.1f;
    static constexpr float SCAN_DELTA_ANGLE_PITCH = 0.1f;

    /**
     * The number of times refresh is called without receiving valid CV data to when
     * the command will consider the target lost and start tracking.
     */
    static constexpr int AIM_LOST_NUM_COUNTS = 500;

    SentinelTurretCVCommand(
        tap::Drivers *drivers,
        tap::control::turret::TurretSubsystemInterface *DoublePitchTurretSubsystem,
        aruwsrc::agitator::AgitatorSubsystem *agitatorSubsystem,
        sentinel::firing::SentinelSwitcherSubsystem *switcher);

    bool isReady() override { return sentinelTurret->isOnline(); }

    void initialize() override;

    bool isFinished() const override { return false; }

    void execute() override;

    void end(bool) override;

    const char *getName() const override { return "sentinel turret CV"; }

    inline bool isAimingAtTarget() const { return aimingAtTarget; }

private:
    static constexpr float YAW_P = 4500.0f;
    static constexpr float YAW_I = 0.0f;
    static constexpr float YAW_D = 140.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
    static constexpr float YAW_MAX_OUTPUT = 32000.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

    static constexpr float PITCH_P = 3500.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 80.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 32000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.5f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;

    tap::Drivers *drivers;

    tap::control::turret::TurretSubsystemInterface *sentinelTurret;

    sentinel::firing::SentinelRotateAgitatorCommand rotateAgitator;

    bool pitchScanningUp;
    bool yawScanningRight;
    bool aimingAtTarget;

    /**
     * A counter that is reset to 0 every time CV starts tracking a target
     * and that keeps track of the number of times `refresh` is called when
     * CV no longer is tracking a target.
     */
    int lostTargetCounter;

    tap::algorithms::SmoothPid yawPid;
    tap::algorithms::SmoothPid pitchPid;

    uint32_t prevTime = 0;

    void scanForTarget();

    /**
     * Updates `axisScanningUp` based on the current setpoint and the min and max scanning
     * setpoints.
     */
    static void updateScanningUp(
        const float motorSetpoint,
        const float minMotorSetpoint,
        const float maxMotorSetpoint,
        bool *axisScanningUp);
};  // class SentinelTurretCVCommand

}  // namespace aruwsrc::control::turret

#endif  // SENTINEL_TURRET_CV_COMMAND_HPP_
