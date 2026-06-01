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
/**
 * @todo: this engineer-specific imu calibrate command adds odometry reset functionality, which
 * should be incorporated into the regular imu calibrate command. After incorporating, this class
 * can be deleted
 */
#include "engineer_imu_calibrate_command.hpp"

#include "tap/drivers.hpp"

#include "aruwsrc/algorithms/binned_encoder_alignment/binned_encoder_alignment.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"

using namespace tap::algorithms;

namespace aruwsrc::engineer
{
EngineerImuCalibrateCommand::EngineerImuCalibrateCommand(
    tap::Drivers *drivers,
    const std::vector<TurretIMUCalibrationConfig> &turretsAndControllers,
    control::chassis::HolonomicChassisSubsystem *chassis,
    float velocityZeroThreshold = ImuCalibrateCommand::DEFAULT_VELOCITY_ZERO_THRESHOLD,
    float positionZeroThreshold = ImuCalibrateCommand::DEFAULT_POSITION_ZERO_THRESHOLD,
    aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver &yawObserver,
    tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
    tap::encoder::EncoderInterface &turretLampreyEncoder,
    tap::encoder::EncoderInterface &turretPulleyEncoder,
    tap::encoder::EncoderInterface &turretInternalEncoder,
    const float binnedAlignmentOffset,
    const float homeAlignmentOffset,
    aruwsrc::control::buzzer::NoteSequenceCommand *successChime,
    aruwsrc::control::buzzer::NoteSequenceCommand *failChime)
    : aruwsrc::control::imu::ImuCalibrateCommand(
          drivers,
          turretsAndControllers,
          chassis,
          EngineerImuCalibrateCommand::VELOCITY_ZERO_THRESHOLD,
          EngineerImuCalibrateCommand::POSITION_ZERO_THRESHOLD,
          successChime,
          failChime,
          &odometryInterface),
      yawObserver(yawObserver),
      odometryInterface(odometryInterface),
      turretLampreyEncoder(turretLampreyEncoder),
      turretPulleyEncoder(turretPulleyEncoder),
      turretInternalEncoder(turretInternalEncoder),
      successChime(successChime),
      failChime(failChime),
      binnedAlignmentOffset(binnedAlignmentOffset),
      homeAlignmentOffset(homeAlignmentOffset),
      fakeLampreyEncoder(0, 0)
{
    for (auto &config : turretsAndControllers)
    {
        addSubsystemRequirement(config.turret);
    }
}

void EngineerImuCalibrateCommand::initialize()
{
    
    ImuCalibrateCommand::initialize();

    calibrationLongTimeout.stop();
    calibrationTimer.stop();
    prevTime = tap::arch::clock::getTimeMilliseconds();
    lampreyAligned = false;
}



void EngineerImuCalibrateCommand::execute()
{
    switch (calibrationState)
    {
        case CalibrationState::WAITING_FOR_SYSTEMS_ONLINE:
        {
            if (calibrationLongTimeout.isExpired())
            {
                if (failChime) drivers->commandScheduler.addCommand(failChime);
                calibrationState = CalibrationState::CALIBRATION_FAIL;
            }
            // Only start calibrating if the turret is online and if there is an IMU online to be
            // calibrated. The onboard Mpu6500 will never be in the `IMU_NOT_CONNECTED` state unless
            // the Mpu6500 is shorted (which has never happened). The turret MCB will only be
            // offline if the turret MCB is unplugged.
            bool turretMCBsReady = true;
            bool turretsOnline = true;

            for (auto &config : turretsAndControllers)
            {
                turretMCBsReady &= config.turretImu->isOnline();
                turretsOnline &= config.turret->isOnline();
            }

            if (turretsOnline && (turretMCBsReady || (drivers->mpu6500.getImuState() !=
                                                      tap::communication::sensors::imu::mpu6500::Mpu6500::ImuState::IMU_NOT_CONNECTED)))
            {
                calibrationLongTimeout.restart(MAX_CALIBRATION_WAITTIME_MS);
                calibrationTimer.restart(WAIT_TIME_TURRET_RESPONSE_MS);
                calibrationState = CalibrationState::LOCKING_TURRET;
            }

            break;
        }
        case CalibrationState::LOCKING_TURRET:
        {
            if (calibrationLongTimeout.isExpired())
            {
                if (failChime) drivers->commandScheduler.addCommand(failChime);
                calibrationState = CalibrationState::CALIBRATION_FAIL;
            }

            bool turretsNotMoving = true;
            for (auto &config : turretsAndControllers)
            {
                turretsNotMoving &=
                    turretReachedCenterAndNotMoving(config.turret, !config.turretImuOnPitch);
            }

            if (calibrationTimer.isExpired() && turretsNotMoving)
            {
                // enter calibration phase
                if (!lampreyAligned)
                {
                    // Preform the binned alignment
                    fakeLampreyEncoder.setFakePosition(
                        aruwsrc::algorithms::binned_encoder_alignment::calculatePosition<30, 95>(
                            turretPulleyEncoder.getPosition().getWrappedValue(),
                            turretLampreyEncoder.getPosition().getWrappedValue(),
                            binnedAlignmentOffset) -
                        homeAlignmentOffset);

                    turretInternalEncoder.alignWith(&fakeLampreyEncoder);
                    lampreyAligned = true;
                    // exit out so we move to the new setpoint
                    return;
                }

                calibrationTimer.stop();
                calibrationLongTimeout.restart(MAX_CALIBRATION_WAITTIME_MS);
                for (auto &config : turretsAndControllers)
                {
                    config.turretImu->requestCalibration();
                }

                calibrationState = CalibrationState::CALIBRATING_IMU;
            }

            break;
        }
        case CalibrationState::CALIBRATING_IMU:
            if (calibrationLongTimeout.isExpired())
            {
                if (failChime) drivers->commandScheduler.addCommand(failChime);
                calibrationState = CalibrationState::CALIBRATION_FAIL;
            }

            if (drivers->mpu6500.getImuState() == tap::communication::sensors::imu::mpu6500::Mpu6500::ImuState::IMU_CALIBRATED)
            {
                // assume turret MCB takes approximately as long as the onboard IMU to calibrate,
                // plus 1 second extra to handle sending the request and processing it
                // TODO to handle the case where the turret MCB doesn't receive information,
                // potentially add ACK sequence to turret MCB CAN comm class.
                calibrationTimer.restart(TURRET_IMU_EXTRA_WAIT_CALIBRATE_MS);
                calibrationState = CalibrationState::WAITING_CALIBRATION_COMPLETE;
                if (odometry2DInterface != nullptr)
                {
                    odometry2DInterface->reset();
                }
            }
            break;
        case CalibrationState::WAITING_CALIBRATION_COMPLETE:
            if (calibrationTimer.isExpired())
            {
                calibrationState = CalibrationState::CALIBRATION_SUCCESS;
                if (successChime) drivers->commandScheduler.addCommand(successChime);
            }
            break;
        default:
            break;
    }

    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    float dt = (currTime - prevTime) / 1000.0f;
    prevTime = currTime;

    for (auto &config : turretsAndControllers)
    {
        // don't run pitch controller when turret IMU not on pitch (as there is no need)
        if (config.turretImuOnPitch)
        {
            config.pitchController->runController(
                dt,
                config.turret->pitchMotor.getChassisFrameSetpoint());
        }
        config.yawController->runController(dt, config.turret->yawMotor.getChassisFrameSetpoint());
    }
}

bool EngineerImuCalibrateCommand::isFinished() const
{
    return calibrationState == CalibrationState::CALIBRATION_SUCCESS ||
           calibrationState == CalibrationState::CALIBRATION_FAIL;
}

}  // namespace aruwsrc::engineer
