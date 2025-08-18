/*
 * Copyright (c) 2020-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

/**
 * @file gravity_autotune.hpp
 *
 * @brief   Implements gravity-based center-of-mass autotuning for turret calibration.
 *
 * Defines the GravityAutotune command, which locks the turret at specified
 * test points, measures torque/angle, and estimates the turret's center of
 * mass using least squares regression.
 */

#ifndef GRAVITY_AUTOTUNE_HPP_
#define GRAVITY_AUTOTUNE_HPP_

#include <Eigen/Dense>

#include "tap/algorithms/ramp.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/robot/standard/standard_turret_subsystem.hpp"

namespace aruwsrc::control::autotune
{
class GravityAutotuneBase : public tap::control::Command
{
public:
    enum class CalibrationState
    {
        WAITING_FOR_SYSTEMS_ONLINE,
        LOCKING_TURRET,
        MEASURING_TORQUE,
        NEXT_LOCATION,
        CALIBRATION_SUCCESS,
        CALIBRATION_FAIL,
        DONE
    };

    virtual ~GravityAutotuneBase() = default;

    virtual CalibrationState getCalibrationState() const = 0;

    virtual std::array<float, 3> getCalibrationResult() const = 0;
};

template <uint32_t numTestPoints>
class GravityAutotune : public GravityAutotuneBase
{
public:
    struct TurretCalibrationConfig
    {
        /// A `TurretSubsystem` that this command will control (will lock the turret).
        turret::TurretSubsystem *turret;
        /// A chassis relative pitch controller used to lock the turret.
        turret::algorithms::ChassisFramePitchTurretController *pitchController;
        /// Mass of the pitching part of the turret in units of Kg
        const float turretMass;
        /// A constant that relates the motor units to Nm of torque, would only work with current
        /// controlled motors. In units of Nm / desOut
        const float torqueToDesiredOut;
        /// Force of gravity. Unlikely to change. m / s^2
        const float gravity = 9.81;
    };

    GravityAutotune(
        tap::Drivers *drivers,
        const TurretCalibrationConfig &config,
        chassis::HolonomicChassisSubsystem *chassis,
        const std::array<float, numTestPoints> points,
        const float velocityZeroThreshold,
        const float positionZeroThreshold,
        aruwsrc::control::buzzer::NoteSequenceCommand *successChime = nullptr,
        aruwsrc::control::buzzer::NoteSequenceCommand *failChime = nullptr);

    /**
     * @brief   Returns the current calibration state.
     * @return  The active CalibrationState.
     */
    GravityAutotuneBase::CalibrationState getCalibrationState() const override
    {
        return calibrationState;
    }

    /**
     * @brief   Retrieves the last computed center of mass calibration result.
     * @return  Array containing {cgX_mm, cgZ_mm, magnitude_desOut}.
     */
    std::array<float, 3> getCalibrationResult() const override { return calibrationResult; }

    void initialize() override;

    void execute() override;

    void end(bool) override;

    virtual bool isFinished() const override;

    const char *getName() const override { return "Gravity Autotune Command"; }

private:
    tap::Drivers *drivers;
    TurretCalibrationConfig config;
    chassis::HolonomicChassisSubsystem *chassis;
    std::array<float, numTestPoints> points;

    const float velocityZeroThreshold;
    const float positionZeroThreshold;

    aruwsrc::control::buzzer::NoteSequenceCommand *successChime;
    aruwsrc::control::buzzer::NoteSequenceCommand *failChime;

    GravityAutotuneBase::CalibrationState calibrationState;

    // Current point in the sequence being measured
    size_t currentPointIndex = 0;

    // Previous time, used for the controller's dt
    uint32_t prevTime = 0;

    // Value to store what sample number we're currently at
    uint32_t samplePointCount = 0;

    // Value to store the averaging torque values
    float averagingTorques = 0;

    // Value to store the averaging angle values
    float averagingAngles = 0;

    // Array of torque measurements received post averaging
    std::array<float, numTestPoints> measuredTorques{};

    // Array of angle measurements received post averaging
    std::array<float, numTestPoints> measuredAngles{};

    /**
     * Amount of time the turret has to have passed `turretReachedPointAndNotMoving()`
     */
    static constexpr uint32_t WAIT_TIME_TURRET_RESPONSE_MS = 1000;

    /**
     * Wait timeout for the command to wait until it gives up.
     * Is a safety precaution to avoid getting stuck in calibration forever.
     */
    static constexpr uint32_t MAX_CALIBRATION_WAITTIME_MS = 1000 * 20;

    /**
     * Number of sample points per test point to average the torque measurement.
     */
    static constexpr uint32_t NUM_SAMPLE_POINTS = 1000;

    /**
     * Timeout that we set after initially starting the turret PID controller to allow any residual
     * movement from starting the new PID controller to be resolved.
     */
    tap::arch::MilliTimeout calibrationTimer;

    /**
     * Timeout used to determine if we should give up on tuning.
     */
    tap::arch::MilliTimeout calibrationLongTimeout;

    /**
     * Place to store the last calibration result
     */
    std::array<float, 3> calibrationResult{};

    /**
     * @brief Calculates the center of mass with least squares
     *
     * @return std::array<float,3> cgX, cgZ, and magnitude of the center of mass
     * with cgX, and cgZ in units of mm and magnitude in units of desOut.
     */
    std::array<float, 3> calculateCOM(
        std::array<float, numTestPoints> Angles,
        std::array<float, numTestPoints> Torques);

    inline bool turretReachedPointAndNotMoving(
        control::turret::TurretSubsystem *turret,
        const WrappedFloat setpoint) const
    {
        return compareFloatClose(
                   0.0f,
                   turret->pitchMotor.getChassisFrameVelocity(),
                   velocityZeroThreshold) &&
               (turret->pitchMotor.getChassisFrameMeasuredAngle().minDifference(setpoint) <
                positionZeroThreshold);
    }

    /**
     * @brief Helper function to check if the safety timer is expired
     */
    inline void checkSafetyTimeout()
    {
        if (calibrationLongTimeout.isExpired())
        {
            if (failChime) drivers->commandScheduler.addCommand(failChime);
            calibrationState = CalibrationState::CALIBRATION_FAIL;
        }
    }

    /**
     * @brief Helper function that turns the calibration result into 
     * units of mm.
     *  
     * @param calibrationNum Value from the COM calculation
     * @return float `COMLocation` in mm
     */
    inline float calibrationResultToMM(float calibrationNum)
    {
        // desOut*m * mm/m * Nm/desOut * s^2/m * 1/kg = mm
        return calibrationNum * 1000 * config.torqueToDesiredOut / config.gravity /
               config.turretMass;
    }

};  // class autotune
}  // namespace aruwsrc::control::autotune

#include "gravity_autotune_impl.hpp"

#endif  // GRAVITY_AUTOTUNE_HPP_