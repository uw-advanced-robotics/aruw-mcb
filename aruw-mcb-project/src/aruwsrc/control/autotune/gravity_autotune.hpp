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
    enum CalibrationState
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

    /**
     * Get the current calibration state.
     */
    virtual CalibrationState getCalibrationState() const = 0;
};

/**
 * A command for automatically tuning a subsystem
 */
template <uint32_t numTestPoints>
class GravityAutotune : public GravityAutotuneBase
{
public:
    using CalibrationState = GravityAutotuneBase::CalibrationState;

    GravityAutotune(
        tap::Drivers *drivers,
        const control::imu::ImuCalibrateCommand::TurretIMUCalibrationConfig &turretAndControllers,
        std::array<float, numTestPoints> points,
        float velocityZeroThreshold =
            control::imu::ImuCalibrateCommand::DEFAULT_VELOCITY_ZERO_THRESHOLD,
        float positionZeroThreshold =
            control::imu::ImuCalibrateCommand::DEFAULT_POSITION_ZERO_THRESHOLD,
        aruwsrc::control::buzzer::NoteSequenceCommand *successChime = nullptr,
        aruwsrc::control::buzzer::NoteSequenceCommand *failChime = nullptr);

    CalibrationState getCalibrationState() const override { return calibrationState; }

    void initialize() override;
    
    void execute() override;
    
    void end(bool) override;
    
    virtual bool isFinished() const override;
    
    const char *getName() const override { return "Autotune subsystem"; }
    
    private:
    tap::Drivers *drivers;
    control::imu::ImuCalibrateCommand::TurretIMUCalibrationConfig turretAndControllers;
    std::array<float, numTestPoints> points;

    float velocityZeroThreshold;
    float positionZeroThreshold;

    aruwsrc::control::buzzer::NoteSequenceCommand *successChime;
    aruwsrc::control::buzzer::NoteSequenceCommand *failChime;

    CalibrationState calibrationState;

    // Current point being measured
    size_t pointMeasuring = 0;
    float setpoint = 0.0f;

    uint32_t prevTime = 0;

    uint32_t samplePointCount = 0;

    // Value to store the averaging torque values
    float torqueMeasurements = 0;

    std::array<float, numTestPoints> measuredTorques{};
    /**
     * Wait a minimum of this time to allow the turret to settle at a locked position (in ms).
     */
    static constexpr uint32_t WAIT_TIME_TURRET_RESPONSE_MS = 1000;
    /**
     * Wait this time after the mpu6500 is done calibrating to ensure the turret MCB's IMU is
     * calibrated.
     */
    static constexpr uint32_t TURRET_IMU_EXTRA_WAIT_CALIBRATE_MS = 2000;

    /**
     * Wait timeout (after state `WAITING_FOR_SYSTEMS_ONLINE` is complete) for the command to wait
     * until it gives up. Should never happen but is a safety precaution to avoid
     * getting stuck in calibration forever.
     */
    static constexpr uint32_t MAX_CALIBRATION_WAITTIME_MS = 1000 * 20;

    /**
     * Number of sample points per test point to average the torque measurement.
     */
    static constexpr uint32_t NUM_SAMPLE_POINTS = 1000;

    /**
     * Timeout that we set after initially starting the turret PID controller to allow any residual
     * movement from starting the new PID controller to be resolved.
     *
     * Also the delay that we set after onboard mpu6500 is calibrated to ensure that turret IMU has
     * enough time to successfully calibrate.
     */
    tap::arch::MilliTimeout calibrationTimer;

    /**
     * Timeout used to determine if we should give up on calibration.
     */
    tap::arch::MilliTimeout calibrationLongTimeout;

    std::array<float,2> calculateCOM();

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

    inline void checkSafetyTimeout()
    {
        if (calibrationLongTimeout.isExpired())
        {
            if (failChime) drivers->commandScheduler.addCommand(failChime);
            calibrationState = CalibrationState::CALIBRATION_FAIL;
        }
    }

};  // class autotune
}  // namespace aruwsrc::control::autotune

#include "gravity_autotune_impl.hpp"

#endif  // GRAVITY_AUTOTUNE_HPP_