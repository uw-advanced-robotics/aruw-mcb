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

#ifndef GRAVITY_AUTOTUNE_IMPL_HPP_
#define GRAVITY_AUTOTUNE_IMPL_HPP_

#include "gravity_autotune.hpp"

namespace aruwsrc::control::autotune
{
template <uint32_t numTestPoints>
GravityAutotune<numTestPoints>::GravityAutotune(
    tap::Drivers *drivers,
    const TurretCalibrationConfig &config,
    chassis::HolonomicChassisSubsystem *chassis,
    const std::array<float, numTestPoints> points,
    const float velocityZeroThreshold,
    const float positionZeroThreshold,
    aruwsrc::control::buzzer::NoteSequenceCommand *successChime,
    aruwsrc::control::buzzer::NoteSequenceCommand *failChime)
    : drivers(drivers),
      config(config),
      chassis(chassis),
      points(points),
      velocityZeroThreshold(velocityZeroThreshold),
      positionZeroThreshold(positionZeroThreshold),
      successChime(successChime),
      failChime(failChime)
{
    addSubsystemRequirement(config.turret);
    addSubsystemRequirement(chassis);
}

template <uint32_t numTestPoints>
void GravityAutotune<numTestPoints>::initialize()
{
    if (chassis != nullptr)
    {
        chassis->setDesiredOutput(0, 0, 0);
    }

    calibrationState = CalibrationState::WAITING_FOR_SYSTEMS_ONLINE;
    calibrationLongTimeout.stop();
    calibrationTimer.stop();
    prevTime = tap::arch::clock::getTimeMilliseconds();

    samplePointCount = 0;
    currentPointIndex = 0;

    config.pitchController->initialize();
    config.turret->pitchMotor.setChassisFrameSetpoint(Angle(points[currentPointIndex]));

    calibrationLongTimeout.restart(MAX_CALIBRATION_WAITTIME_MS);
    calibrationTimer.restart(WAIT_TIME_TURRET_RESPONSE_MS);
}

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
 *   over the period of the `calibrationLongTimeout` to ensure the user can regain
 *   control.
 */
template <uint32_t numTestPoints>
void GravityAutotune<numTestPoints>::execute()
{
    switch (calibrationState)
    {
        case CalibrationState::WAITING_FOR_SYSTEMS_ONLINE:
        {
            const bool turretsOnline = config.turret->isOnline();

            // Calibration timer to give people a chance to move out of the way
            if (turretsOnline && calibrationTimer.execute())
            {
                calibrationLongTimeout.restart(MAX_CALIBRATION_WAITTIME_MS);
                calibrationTimer.restart(WAIT_TIME_TURRET_RESPONSE_MS);
                calibrationState = CalibrationState::LOCKING_TURRET;
            }
        }
        break;
        case CalibrationState::LOCKING_TURRET:
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
                calibrationState = CalibrationState::MEASURING_TORQUE;
                samplePointCount = 0;
            }
        }
        break;

        case CalibrationState::MEASURING_TORQUE:
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
                calibrationState = CalibrationState::NEXT_LOCATION;

                // Finished going through all points
                if (currentPointIndex == points.size())
                {
                    calibrationState = CalibrationState::DONE;
                }
            }
        }
        break;

        case CalibrationState::NEXT_LOCATION:
        {
            config.turret->pitchMotor.setChassisFrameSetpoint(Angle(points[currentPointIndex]));
            calibrationLongTimeout.restart(MAX_CALIBRATION_WAITTIME_MS);
            calibrationTimer.restart(WAIT_TIME_TURRET_RESPONSE_MS);
            calibrationState = CalibrationState::LOCKING_TURRET;
        }
        break;

        case CalibrationState::DONE:
        {
            // Turn off in case calculation takes awhile
            config.turret->yawMotor.setMotorOutput(0);
            config.turret->pitchMotor.setMotorOutput(0);
            calibrationState = CalibrationState::CALIBRATION_SUCCESS;
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

    config.pitchController->runController(dt, config.turret->pitchMotor.getChassisFrameSetpoint());
}

template <uint32_t numTestPoints>
void GravityAutotune<numTestPoints>::end(bool)
{
    switch (calibrationState)
    {
        case CalibrationState::CALIBRATION_SUCCESS:
        {
            calibrationResult = calculateCOM(measuredAngles, measuredTorques);
            if (successChime) drivers->commandScheduler.addCommand(successChime);
        }
        break;

        case CalibrationState::CALIBRATION_FAIL:
        {
            if (failChime) drivers->commandScheduler.addCommand(failChime);
        }
        break;

        default:
            break;
    }
}

template <uint32_t numTestPoints>
bool GravityAutotune<numTestPoints>::isFinished() const
{
    return calibrationState == CalibrationState::CALIBRATION_SUCCESS ||
           calibrationState == CalibrationState::CALIBRATION_FAIL;
}

template <uint32_t numTestPoints>
std::array<float, 3> GravityAutotune<numTestPoints>::calculateCOM(
    std::array<float, numTestPoints> Angles,
    std::array<float, numTestPoints> Torques)
{
    Eigen::MatrixXd X(numTestPoints, 2);
    Eigen::VectorXd Y(numTestPoints);

    for (uint32_t i = 0; i < numTestPoints; ++i)
    {
        X(i, 0) = std::cos(Angles[i]);  // corresponds to A (m·g·x)
        X(i, 1) = std::sin(Angles[i]);  // corresponds to B (−m·g·z)
        Y(i) = Torques[i];
    }
    // Solve least squares: torque = A·cos(theta) + B·sin(theta)
    Eigen::Vector2d params = X.colPivHouseholderQr().solve(Y);

    const float A = params(0);
    const float B = params(1);
    const float magnitude = std::sqrt(A * A + B * B);

    return {calibrationResultToMM(A), calibrationResultToMM(B), magnitude};
}

}  // namespace aruwsrc::control::autotune

#endif  // GRAVITY_AUTOTUNE_IMPL_HPP_