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
 * Defines the GravityAutotuneCommand command, which locks the turret at specified
 * test points, measures torque/angle, and estimates the turret's center of
 * mass using least squares regression.
 */

#ifndef AUTOTUNE_COMMAND_INTERFACE_HPP_
#define AUTOTUNE_COMMAND_INTERFACE_HPP_

#include <Eigen/Dense>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/buzzer/note_sequence_command.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"

namespace aruwsrc::control::autotune
{
/** @brief Non-template class to allow for getting the gravity autotune commands
 * in a weak function, as used in the gravity autotune menu.
 */
class TurretAutotuneInterface : public tap::control::Command
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

    virtual ~TurretAutotuneInterface() = default;

    virtual CalibrationState getCalibrationState() const = 0;

    virtual const char *getName() const = 0;

    virtual void drawCalibrationResult(modm::GraphicDisplay &display) const = 0;
};

template <typename T, uint32_t numTestPoints>
class TurretAutotuneCommand : public TurretAutotuneInterface
{
public:
    /**  @brief Turret calibration config struct, all default members are not necessary required.
     *  They are only important for giving a result with accurate units, as the gravity compensation
     *  converts the cx and cz into a unit vector multiplied by the compensation scalar it doesn't
     * require real units.
     */
    struct TurretCalibrationConfig
    {
        /// A `TurretSubsystem` that this command will control (will lock the turret).
        turret::TurretSubsystem *turret;
        /// A chassis relative pitch controller used to lock the turret.
        turret::algorithms::ChassisFrameTurretController<turret::algorithms::Axis::PITCH>
            *pitchController;
        /// If the pitch motor is inverted
        bool isMotorInverted;
        /// Mass of the pitching part of the turret in units of Kg
        float turretMass = 1.0f;
        /// A constant that relates the motor units to Nm of torque, would only work with current
        /// controlled motors. In units of Nm / desOut
        float torqueToDesiredOut = 1.0f;
        /// Force of gravity. Unlikely to change. m / s^2
        const float gravity = ACCELERATION_GRAVITY;
    };

    TurretAutotuneCommand(
        tap::Drivers *drivers,
        const TurretCalibrationConfig &config,
        chassis::HolonomicChassisSubsystem *chassis = nullptr,
        const std::array<float, numTestPoints> points = {},
        const float velocityZeroThreshold = DEFAULT_VELOCITY_THRESHOLD,
        const float positionZeroThreshold = DEFAULT_POSITION_THRESHOLD,
        aruwsrc::control::buzzer::NoteSequenceCommand *successChime = nullptr,
        aruwsrc::control::buzzer::NoteSequenceCommand *failChime = nullptr)
        : drivers(drivers),
          config(config),
          chassis(chassis),
          points(points),
          velocityZeroThreshold(velocityZeroThreshold),
          positionZeroThreshold(positionZeroThreshold),
          successChime(successChime),
          failChime(failChime)
    {
        this->addSubsystemRequirement(config.turret);

        if (chassis != nullptr)
        {
            this->addSubsystemRequirement(chassis);
        }

        // Fill the points array with evenly spaced points if the array is all zeros
        bool allZero = std::all_of(this->points.begin(), this->points.end(), [](float v) {
            return v == 0.0f;
        });

        if (allZero)
        {
            const float minAngle = config.turret->pitchMotor.getConfig().minAngle;
            const float maxAngle = config.turret->pitchMotor.getConfig().maxAngle;

            for (size_t i = 0; i < numTestPoints; ++i)
            {
                this->points[i] = minAngle + i * (maxAngle - minAngle) / (numTestPoints - 1);
            }
        }
    }

    virtual T calculate(
        std::array<float, numTestPoints> Angles,
        std::array<float, numTestPoints> Torques) const = 0;

    /**
     * @brief   Returns the current calibration state.
     * @return  The active CalibrationState.
     */
    TurretAutotuneInterface::CalibrationState getCalibrationState() const override
    {
        return calibrationState;
    }

    /**
     * @brief Initializes the autotune command, resetting state and timers.
     */
    void initialize() override
    {
        if (chassis != nullptr)
        {
            chassis->setDesiredOutput(0, 0, 0);
        }

        calibrationState = TurretAutotuneInterface::CalibrationState::WAITING_FOR_SYSTEMS_ONLINE;
        calibrationFailTimeout.stop();
        calibrationTimer.stop();
        prevTime = tap::arch::clock::getTimeMilliseconds();

        samplePointCount = 0;
        currentPointIndex = 0;

        config.pitchController->initialize();
        config.turret->pitchMotor.setChassisFrameSetpoint(Angle(points[currentPointIndex]));

        calibrationFailTimeout.restart(MAX_CALIBRATION_WAITTIME_MS);
        calibrationTimer.restart(WAIT_TIME_TURRET_RESPONSE_MS);
    };

    /**
     * @brief Executes one cycle of the gravity calibration state machine.
     *
     * This method drives the calibration process by moving the turret to
     * predetermined angles, measuring motor torque at each point, and determining
     * when calibration is complete.
     *
     * @details
     * The calibration process is implemented as a state machine with the following states:
     *
     * - **WAITING_FOR_SYSTEMS_ONLINE**
     *   Waits until the turret is online and a short wait timer expires.
     *   Once ready, restarts a long calibration timeout and transitions to
     *   `LOCKING_TURRET`.
     *
     * - **LOCKING_TURRET**
     *   Waits until the turret has reached its target angle and is no longer moving.
     *   If movement is detected, restarts the short wait timer.
     *   When stable and the timer expires, transitions to `MEASURING_TORQUE`.
     *
     * - **MEASURING_TORQUE**
     *   Collects a fixed number of torque and location samples from the turret's pitch motor,
     *   averaging.
     *   After samples are collected:
     *     - If more points remain, transitions to `NEXT_LOCATION`.
     *     - If all points are measured, transitions to `DONE`.
     *
     * - **NEXT_LOCATION**
     *   Moves the turret to the next target angle for measurement,
     *   restarts the long calibration timeout, and returns to `LOCKING_TURRET`.
     *
     * - **DONE**
     *   Turns off the motors as to be sure that in the case of the robot hanging during
     *   the calculation it won't become uncontrolled (should never happen) and transitions
     *   into `CALIBRATION_SUCCESS`
     *
     * - **CALIBRATION_FAIL**
     *   Calibration fail is called if the turret is unable to lock at a single position
     *   over the period of the `calibrationFailTimeout` to ensure the user can regain
     *   control.
     */
    void execute() override
    {
        switch (calibrationState)
        {
            case TurretAutotuneInterface::CalibrationState::WAITING_FOR_SYSTEMS_ONLINE:
            {
                bool allOnline = true;
                const bool turretsOnline = config.turret->isOnline();

                if (chassis != nullptr)
                {
                    allOnline &= chassis->allMotorsOnline();
                }

                allOnline &= turretsOnline;

                // Calibration timer to give people a chance to move out of the way
                if (allOnline && calibrationTimer.execute())
                {
                    calibrationFailTimeout.restart(MAX_CALIBRATION_WAITTIME_MS);
                    calibrationTimer.restart(WAIT_TIME_TURRET_RESPONSE_MS);
                    calibrationState = TurretAutotuneInterface::CalibrationState::LOCKING_TURRET;
                }
            }
            break;
            case TurretAutotuneInterface::CalibrationState::LOCKING_TURRET:
            {
                const bool turretNotMoving = turretReachedPointAndNotMoving(
                    config.turret,
                    config.turret->pitchMotor.getChassisFrameSetpoint());

                if (!turretNotMoving)
                {
                    calibrationTimer.restart(WAIT_TIME_TURRET_RESPONSE_MS);
                }

                // Exit Locking Turret
                if (calibrationTimer.isExpired() && turretNotMoving)
                {
                    calibrationState = TurretAutotuneInterface::CalibrationState::MEASURING_TORQUE;
                    samplePointCount = 0;
                }
            }
            break;

            case TurretAutotuneInterface::CalibrationState::MEASURING_TORQUE:
            {
                if (samplePointCount < NUM_SAMPLE_POINTS)
                {
                    // Increment sample point first so it's not 0 when first average
                    samplePointCount++;

                    // Add to the running average of the motors value and angle measurements
                    const float motorValue =
                        static_cast<float>(config.turret->pitchMotor.getMotorOutput());
                    averagingTorques += (motorValue - averagingTorques) / (samplePointCount);

                    const float angleValue =
                        config.turret->pitchMotor.getChassisFrameMeasuredAngle().getWrappedValue();
                    averagingAngles += (angleValue - averagingAngles) / samplePointCount;
                }
                else
                {
                    // Store averaged values
                    measuredTorques[currentPointIndex] = averagingTorques;
                    measuredAngles[currentPointIndex] = averagingAngles;

                    // Switch to next point and reset averaging
                    currentPointIndex++;
                    averagingTorques = 0;
                    averagingAngles = 0;
                    samplePointCount = 0;

                    // Exit measuring when done taking samples
                    calibrationState = TurretAutotuneInterface::CalibrationState::NEXT_LOCATION;

                    // Finished going through all points
                    if (currentPointIndex == points.size())
                    {
                        calibrationState = TurretAutotuneInterface::CalibrationState::DONE;
                    }
                }
            }
            break;

            case TurretAutotuneInterface::CalibrationState::NEXT_LOCATION:
            {
                config.turret->pitchMotor.setChassisFrameSetpoint(Angle(points[currentPointIndex]));
                calibrationFailTimeout.restart(MAX_CALIBRATION_WAITTIME_MS);
                calibrationTimer.restart(WAIT_TIME_TURRET_RESPONSE_MS);
                calibrationState = TurretAutotuneInterface::CalibrationState::LOCKING_TURRET;
            }
            break;

            case TurretAutotuneInterface::CalibrationState::DONE:
            {
                // Turn off in case calculation takes awhile
                config.turret->yawMotor.setMotorOutput(0);
                config.turret->pitchMotor.setMotorOutput(0);
                calibrationState = TurretAutotuneInterface::CalibrationState::CALIBRATION_SUCCESS;
            }
            break;

            default:
                break;
        }

        checkSafetyTimeout();
        uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
        // Have to use ms to share turret controller
        float dt = (currTime - prevTime);
        prevTime = currTime;

        config.pitchController->runController(
            dt,
            config.turret->pitchMotor.getChassisFrameSetpoint());
    };

    void end(bool) override
    {
        switch (calibrationState)
        {
            case TurretAutotuneInterface::CalibrationState::CALIBRATION_SUCCESS:
            {
                calibrationResult = calculate(measuredAngles, measuredTorques);
                if (successChime) drivers->commandScheduler.addCommand(successChime);
            }
            break;
            default:
                if (failChime) drivers->commandScheduler.addCommand(failChime);
                break;
        }
    };

    bool isFinished() const override
    {
        return calibrationState == TurretAutotuneInterface::CalibrationState::CALIBRATION_SUCCESS ||
               calibrationState == TurretAutotuneInterface::CalibrationState::CALIBRATION_FAIL;
    }

    std::array<float, numTestPoints> getMeasuredAngles() const { return measuredAngles; }
    std::array<float, numTestPoints> getMeasuredTorques() const { return measuredTorques; }

    /**
     * @brief   Retrieves the last computed center of mass calibration result.
     * @return  Array containing {cgX_mm, cgZ_mm, magnitude_desOut}.
     */
    T getCalibrationResult() const { return calibrationResult; }

private:
    tap::Drivers *drivers;
    TurretCalibrationConfig config;
    chassis::HolonomicChassisSubsystem *chassis;
    std::array<float, numTestPoints> points;

    const float velocityZeroThreshold;
    const float positionZeroThreshold;

    aruwsrc::control::buzzer::NoteSequenceCommand *successChime;
    aruwsrc::control::buzzer::NoteSequenceCommand *failChime;

    TurretAutotuneInterface::CalibrationState calibrationState;

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
        if (calibrationFailTimeout.isExpired())
        {
            if (failChime) drivers->commandScheduler.addCommand(failChime);
            calibrationState = TurretAutotuneInterface::CalibrationState::CALIBRATION_FAIL;
        }
    }

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

    /**
     * Timeout that we set after initially starting the turret PID controller to allow any residual
     * movement from starting the new PID controller to be resolved.
     */
    tap::arch::MilliTimeout calibrationTimer;

    /**
     * Timeout used to determine if we should give up on tuning.
     */
    tap::arch::MilliTimeout calibrationFailTimeout;

protected:
    // Thresholds to determine if the turret is "not moving" and "at position"
    static constexpr float DEFAULT_POSITION_THRESHOLD = modm::toRadian(3);
    static constexpr float DEFAULT_VELOCITY_THRESHOLD = modm::toRadian(1e-4f);

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
    static constexpr uint32_t NUM_SAMPLE_POINTS = 2000;

    /**
     * Place to store the last calibration result
     */
    T calibrationResult{};
};  // class autotune
}  // namespace aruwsrc::control::autotune

#endif  // AUTOTUNE_COMMAND_INTERFACE_HPP_