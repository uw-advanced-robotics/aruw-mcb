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

#ifndef GRAVITY_AUTOTUNE_IMPL
#define GRAVITY_AUTOTUNE_IMPL

#include "gravity_autotune.hpp"

namespace aruwsrc::control::autotune
{
template <uint32_t numTestPoints>
GravityAutotune<numTestPoints>::GravityAutotune(
    tap::Drivers *drivers,
    const control::imu::ImuCalibrateCommand::TurretIMUCalibrationConfig &turretAndControllers,
    std::array<float, numTestPoints> points,
    float velocityZeroThreshold,
    float positionZeroThreshold,
    aruwsrc::control::buzzer::NoteSequenceCommand *successChime,
    aruwsrc::control::buzzer::NoteSequenceCommand *failChime)
    : drivers(drivers),
      turretAndControllers(turretAndControllers),
      points(points),
      velocityZeroThreshold(velocityZeroThreshold),
      positionZeroThreshold(positionZeroThreshold),
      successChime(successChime),
      failChime(failChime)
{
    addSubsystemRequirement(turretAndControllers.turret);
}

template <uint32_t numTestPoints>
void GravityAutotune<numTestPoints>::initialize()
{
    calibrationState = CalibrationState::WAITING_FOR_SYSTEMS_ONLINE;
    calibrationLongTimeout.stop();
    calibrationTimer.stop();
    prevTime = tap::arch::clock::getTimeMilliseconds();

    samplePointCount = 0;
    pointMeasuring = 0;

    turretAndControllers.pitchController->initialize();
    turretAndControllers.turret->pitchMotor.setChassisFrameSetpoint(Angle(points[pointMeasuring]));

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
 *   Collects a fixed number of torque samples from the turret's pitch motor,
 *   averaging.
 *   After samples are collected:
 *     - If more points remain, transitions to `NEXT_LOCATION`.
 *     - If all points are measured, transitions to `CALIBRATION_SUCCESS`.
 *
 * - **NEXT_LOCATION**
 *   Moves the turret to the next target angle for measurement,
 *   restarts the long calibration timeout, and returns to `LOCKING_TURRET`.
 */
template <uint32_t numTestPoints>
void GravityAutotune<numTestPoints>::execute()
{
    switch (calibrationState)
    {
        case CalibrationState::WAITING_FOR_SYSTEMS_ONLINE:
        {
            checkSafetyTimeout();
            const bool turretsOnline = turretAndControllers.turret->isOnline();

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
            checkSafetyTimeout();
            const bool turretNotMoving = turretReachedPointAndNotMoving(
                turretAndControllers.turret,
                turretAndControllers.turret->pitchMotor.getChassisFrameSetpoint());

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
            checkSafetyTimeout();

            if (samplePointCount < NUM_SAMPLE_POINTS)
            {
                samplePointCount++;
                const float value =
                    static_cast<float>(turretAndControllers.turret->pitchMotor.getMotorOutput());

                torqueMeasurements += (value - torqueMeasurements) / (samplePointCount);
            }
            else
            {
                // Exit measuring when done taking samples
                calibrationState = CalibrationState::NEXT_LOCATION;
                measuredTorques[pointMeasuring] = torqueMeasurements;
                samplePointCount = 0;

                // Finished going through all points
                if (pointMeasuring == points.size() - 1)
                {
                    calibrationState = CalibrationState::CALIBRATION_SUCCESS;
                }
            }
        }
        break;

        case CalibrationState::NEXT_LOCATION:
        {
            checkSafetyTimeout();
            pointMeasuring++;
            turretAndControllers.turret->pitchMotor.setChassisFrameSetpoint(
                Angle(points[pointMeasuring]));
            calibrationLongTimeout.restart(MAX_CALIBRATION_WAITTIME_MS);
            calibrationState = CalibrationState::LOCKING_TURRET;
        }
        break;

        default:
            break;
    }

    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    float dt = (currTime - prevTime);  // Have to use ms to share turret controller
    prevTime = currTime;

    turretAndControllers.pitchController->runController(
        dt,
        turretAndControllers.turret->pitchMotor.getChassisFrameSetpoint());
}

template <uint32_t numTestPoints>
void GravityAutotune<numTestPoints>::end(bool)
{
    turretAndControllers.turret->yawMotor.setMotorOutput(0);
    turretAndControllers.turret->pitchMotor.setMotorOutput(0);

    calculateCOM();

    if (calibrationState == CalibrationState::CALIBRATION_SUCCESS && successChime)
        drivers->commandScheduler.addCommand(successChime);
    if (calibrationState == CalibrationState::CALIBRATION_FAIL && failChime)
        drivers->commandScheduler.addCommand(failChime);
}

template <uint32_t numTestPoints>
bool GravityAutotune<numTestPoints>::isFinished() const
{
    return calibrationState == CalibrationState::CALIBRATION_SUCCESS ||
           calibrationState == CalibrationState::CALIBRATION_FAIL;
}

template <uint32_t numTestPoints>
std::array<float, 2> GravityAutotune<numTestPoints>::calculateCOM()
{
    Eigen::MatrixXd X(numTestPoints, 2);
    Eigen::VectorXd Y(2);

    for (int i = 0; i < numTestPoints; ++i)
    {
        float theta = points[i];
        X(i, 0) = std::sin(theta);  // corresponds to C (m·g·x)
        X(i, 1) = std::cos(theta);  // corresponds to D (−m·g·y)
        Y(i) = measuredTorques[i];
    }
    Eigen::Vector2d params = X.colPivHouseholderQr().solve(Y);
    float C = params(0);
    float D = params(1);

    return {C / 9.81, D / 9.81};
}

}  // namespace aruwsrc::control::autotune

#endif  // GRAVITY_AUTOTUNE_IMPL