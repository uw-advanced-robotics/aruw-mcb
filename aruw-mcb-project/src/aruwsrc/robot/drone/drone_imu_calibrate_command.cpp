/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "drone_imu_calibrate_command.hpp"

#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"

using tap::algorithms::Angle;

namespace aruwsrc::drone
{
using tap::communication::sensors::imu::ImuInterface;

DroneImuCalibrateCommand::DroneImuCalibrateCommand(
    tap::Drivers &drivers,
    DroneTurretSubsystem &turret,
    tap::communication::sensors::imu::AbstractIMU &turretImu,
    tap::algorithms::SmoothPid &yawPid,
    tap::algorithms::SmoothPid &pitchPid,
    aruwsrc::control::buzzer::NoteSequenceCommand *successChime,
    aruwsrc::control::buzzer::NoteSequenceCommand *failChime)
    : drivers(drivers),
      turret(turret),
      turretImu(turretImu),
      yawPid(yawPid),
      pitchPid(pitchPid),
      successChime(successChime),
      failChime(failChime)
{
    addSubsystemRequirement(&turret);
}

bool DroneImuCalibrateCommand::isReady() { return true; }

void DroneImuCalibrateCommand::initialize()
{
    calibrationState = CalibrationState::WAITING_FOR_SYSTEMS_ONLINE;
    yawPid.reset();
    pitchPid.reset();

    turret.yawMotor.setChassisFrameSetpoint(Angle(turret.yawMotor.getConfig().minAngle));
    turret.pitchMotor.setChassisFrameSetpoint(Angle(turret.pitchMotor.getConfig().maxAngle));

    calibrationTimer.stop();
    calibrationLongTimeout.restart(MAX_CALIBRATION_WAITTIME_MS);
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void DroneImuCalibrateCommand::execute()
{
    const uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    const float dt = static_cast<float>(currTime - prevTime) / 1000.0f;
    prevTime = currTime;

    switch (calibrationState)
    {
        case CalibrationState::WAITING_FOR_SYSTEMS_ONLINE:
            if (calibrationLongTimeout.isExpired())
            {
                failCalibration();
                break;
            }

            if (systemsOnline())
            {
                calibrationTimer.restart(WAIT_TIME_TURRET_RESPONSE_MS);
                calibrationLongTimeout.restart(MAX_CALIBRATION_WAITTIME_MS);
                calibrationState = CalibrationState::LOCKING_TURRET;
            }
            break;

        case CalibrationState::LOCKING_TURRET:
            if (calibrationLongTimeout.isExpired())
            {
                failCalibration();
                break;
            }

            if (calibrationTimer.isExpired() && turretLockedAtCalibrationPosition())
            {
                turretImu.requestCalibration();
                drivers.mpu6500.requestCalibration();
                calibrationState = CalibrationState::CALIBRATING_IMUS;
            }
            break;

        case CalibrationState::CALIBRATING_IMUS:
            if (calibrationLongTimeout.isExpired())
            {
                failCalibration();
                break;
            }

            if (imusCalibrated())
            {
                calibrationTimer.restart(IMU_EXTRA_WAIT_CALIBRATE_MS);
                calibrationState = CalibrationState::WAITING_CALIBRATION_COMPLETE;
            }
            break;

        case CalibrationState::WAITING_CALIBRATION_COMPLETE:
            if (calibrationTimer.isExpired())
            {
                calibrationState = CalibrationState::CALIBRATION_SUCCESS;
                if (successChime != nullptr)
                {
                    drivers.commandScheduler.addCommand(successChime);
                }
            }
            break;

        default:
            break;
    }

    if (calibrationState == CalibrationState::LOCKING_TURRET ||
        calibrationState == CalibrationState::CALIBRATING_IMUS ||
        calibrationState == CalibrationState::WAITING_CALIBRATION_COMPLETE)
    {
        runTurretLock(dt);
    }
}

void DroneImuCalibrateCommand::end(bool)
{
    turret.yawMotor.setMotorOutput(0.0f);
    turret.pitchMotor.setMotorOutput(0.0f);
}

bool DroneImuCalibrateCommand::isFinished() const
{
    return calibrationState == CalibrationState::CALIBRATION_SUCCESS ||
           calibrationState == CalibrationState::CALIBRATION_FAIL;
}

void DroneImuCalibrateCommand::runTurretLock(float dt)
{
    turret.yawMotor.setChassisFrameSetpoint(Angle(turret.yawMotor.getConfig().minAngle));
    turret.pitchMotor.setChassisFrameSetpoint(Angle(turret.pitchMotor.getConfig().maxAngle));

    const float yawOutput = yawPid.runController(
        turret.yawMotor.getValidChassisMeasurementError(),
        turret.yawMotor.getChassisFrameVelocity(),
        dt);
    const float pitchOutput = pitchPid.runController(
        turret.pitchMotor.getValidChassisMeasurementError(),
        turret.pitchMotor.getChassisFrameVelocity(),
        dt);

    turret.yawMotor.setMotorOutput(yawOutput);
    turret.pitchMotor.setMotorOutput(pitchOutput);
}

bool DroneImuCalibrateCommand::systemsOnline() const
{
    return turret.yawMotor.isOnline() && turret.pitchMotor.isOnline() && turretImu.isOnline() &&
           drivers.mpu6500.isOnline();
}

bool DroneImuCalibrateCommand::turretLockedAtCalibrationPosition() const
{
    return tap::algorithms::compareFloatClose(
               0.0f,
               turret.yawMotor.getChassisFrameVelocity(),
               VELOCITY_ZERO_THRESHOLD) &&
           tap::algorithms::compareFloatClose(
               0.0f,
               turret.pitchMotor.getChassisFrameVelocity(),
               VELOCITY_ZERO_THRESHOLD) &&
           fabsf(turret.pitchMotor.getValidChassisMeasurementError()) < POSITION_LOCK_THRESHOLD;
}

bool DroneImuCalibrateCommand::imusCalibrated() const
{
    return turretImu.getImuState() == ImuInterface::ImuState::IMU_CALIBRATED &&
           drivers.mpu6500.getImuState() == ImuInterface::ImuState::IMU_CALIBRATED;
}

void DroneImuCalibrateCommand::failCalibration()
{
    calibrationState = CalibrationState::CALIBRATION_FAIL;
    if (failChime != nullptr)
    {
        drivers.commandScheduler.addCommand(failChime);
    }
}
}  // namespace aruwsrc::drone
