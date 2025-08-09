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

#include "aruwsrc/control/autotune/gravity_autotune.hpp"

#include "aruwsrc/control/turret/turret_subsystem.hpp"

using namespace aruwsrc::control::autotune;

template <uint32_t numTestPoints, uint8_t Turrets>
GravityAutotune<numTestPoints, Turrets>::GravityAutotune(
    tap::Drivers *drivers,
    const std::array<control::imu::ImuCalibrateCommand::TurretIMUCalibrationConfig, Turrets>
        &turretsAndControllers,
    std::array<float, numTestPoints> points,
    float velocityZeroThreshold,
    float positionZeroThreshold,
    aruwsrc::control::buzzer::NoteSequenceCommand *successChime,
    aruwsrc::control::buzzer::NoteSequenceCommand *failChime)
    : tap::control::Command(),
      drivers(drivers),
      turretsAndControllers(turretsAndControllers),
      points(points),
      velocityZeroThreshold(velocityZeroThreshold),
      positionZeroThreshold(positionZeroThreshold),
      successChime(successChime),
      failChime(failChime)
{
}

template <uint32_t numTestPoints, uint8_t Turrets>
void GravityAutotune<numTestPoints, Turrets>::initialize()
{
    samplePointCount = 0;
    pointMeasuring = 0;
    for (auto &config : turretsAndControllers)
    {
        config.pitchController->initialize();
        config.turret->pitchMotor.setChassisFrameSetpoint(points[pointMeasuring]);
    }
}

template <uint32_t numTestPoints, uint8_t Turrets>
void GravityAutotune<numTestPoints, Turrets>::execute()
{
    switch (calibrationState)
    {
        case CalibrationState::WAITING_FOR_SYSTEMS_ONLINE:
        {
            checkSafetyTimeout();

            bool turretsOnline = true;

            for (auto &config : turretsAndControllers)
            {
                turretsOnline &= config.turret->isOnline();
            }
            // wait a bit for the calibration timer to give people a chance to move out of the way
            // of the turret
            if (turretsOnline && calibrationTimer.execute())
            {
                calibrationLongTimeout.restart(MAX_CALIBRATION_WAITTIME_MS);
                calibrationTimer.restart(WAIT_TIME_TURRET_RESPONSE_MS);
                calibrationState = CalibrationState::LOCKING_TURRET;
            }

            break;
        }
        case CalibrationState::LOCKING_TURRET:
        {
            checkSafetyTimeout();

            bool turretsNotMoving = true;

            // set setpoint to go to
            for (auto &config : turretsAndControllers)
            {
                turretsNotMoving &= turretReachedPointAndNotMoving(
                    config.turret,
                    config.turret->pitchMotor.getChassisFrameSetpoint());
            }

            // requires you to be still for two seconds before entering calibration
            if (!turretsNotMoving)
            {
                calibrationTimer.restart(WAIT_TIME_TURRET_RESPONSE_MS);
            }

            if (calibrationTimer.isExpired() && turretsNotMoving)
            {
                // enter calibration phase
                calibrationState = CalibrationState::MEASURING_TORQUE;
                samplePointCount = 0;
            }
        }
        break;

        case CalibrationState::NEXT_LOCATION:
        {
            checkSafetyTimeout();
            pointMeasuring++;

            for (auto &config : turretsAndControllers)
            {
                config.turret->pitchMotor.setChassisFrameSetpoint(points[pointMeasuring]);
            }

            calibrationState = CalibrationState::LOCKING_TURRET;
        }
        break;
        case CalibrationState::MEASURING_TORQUE:
        {
            checkSafetyTimeout();

            for (size_t j = 0; j < turretsAndControllers.size(); ++j)
            {
                std::array<float, Turrets> torqueMeasurementTurret{};
                if (samplePointCount < NUM_SAMPLE_POINTS)
                {
                    torqueMeasurements[pointMeasuring][j] +=
                        static_cast<float>(
                            turretsAndControllers[j].turret->pitchMotor.getMotorOutput()) /
                        NUM_SAMPLE_POINTS;
                    samplePointCount++;
                }
                else
                {
                    calibrationState = CalibrationState::LOCKING_TURRET;

                    if (pointMeasuring >= points.size() - 1)
                    {
                        // If we have reached the end of the test points, we can stop measuring
                        calibrationState = CalibrationState::CALIBRATION_SUCCESS;
                    }
                    samplePointCount = 0;
                }
            }
        }
        break;
        default:
            break;
    }

    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    float dt = (currTime - prevTime);  // Have to use ms to be compatible with the turret controller
    prevTime = currTime;

    for (auto &config : turretsAndControllers)
    {
        config.pitchController->runController(
            dt,
            config.turret->pitchMotor.getChassisFrameSetpoint());
    }
};

template <uint32_t numTestPoints, uint8_t Turrets>
void GravityAutotune<numTestPoints, Turrets>::end(bool)
{
    for (auto &config : turretsAndControllers)
    {
        config.turret->yawMotor.setMotorOutput(0);
        config.turret->pitchMotor.setMotorOutput(0);
    }
    if (calibrationState == CalibrationState::CALIBRATION_SUCCESS && successChime)
        drivers->commandScheduler.addCommand(successChime);
    if (calibrationState == CalibrationState::CALIBRATION_FAIL && failChime)
        drivers->commandScheduler.addCommand(failChime);
}

template <uint32_t numTestPoints, uint8_t Turrets>
bool GravityAutotune<numTestPoints, Turrets>::isFinished() const
{
    return calibrationState == CalibrationState::CALIBRATION_SUCCESS ||
           calibrationState == CalibrationState::CALIBRATION_FAIL;
}