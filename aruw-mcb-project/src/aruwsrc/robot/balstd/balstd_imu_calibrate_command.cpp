/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "balstd_imu_calibrate_command.hpp"

#include "tap/communication/sensors/imu/abstract_imu.hpp"

#include "aruwsrc/control/turret/constants/turret_constants.hpp"

using namespace tap::algorithms;
using namespace tap::communication::sensors::imu::mpu6500;
using namespace tap::communication::sensors::imu;

namespace aruwsrc::balstd
{
BalstdImuCalibrateCommand::BalstdImuCalibrateCommand(
    aruwsrc::balstd::Drivers *drivers,
    const std::vector<TurretIMUCalibrationConfig> &turretsAndControllers,
    chassis::BalstdChassisSubsystem *chassis,
    float velocityZeroThreshold,
    float positionZeroThreshold)
    : tap::control::Command(),
      velocityZeroThreshold(velocityZeroThreshold),
      positionZeroThreshold(positionZeroThreshold),
      drivers(drivers),
      turretsAndControllers(turretsAndControllers),
      chassis(chassis)
{
    for (auto &config : turretsAndControllers)
    {
        assert(config.turretMCBCanComm != nullptr);
        assert(config.turret != nullptr);
        assert(config.yawController != nullptr);
        assert(config.pitchController != nullptr);

        addSubsystemRequirement(config.turret);
    }

    addSubsystemRequirement(chassis);
}

bool BalstdImuCalibrateCommand::isReady() { return true; }

void BalstdImuCalibrateCommand::initialize()
{
    calibrationState = CalibrationState::WAITING_FOR_SYSTEMS_ONLINE;

    if (chassis != nullptr)
    {
        chassis->setZeroRPM();
    }

    for (auto &config : turretsAndControllers)
    {
        config.turret->yawMotor.setChassisFrameSetpoint(
            Angle(config.turret->yawMotor.getConfig().startAngle));
        config.turret->pitchMotor.setChassisFrameSetpoint(
            Angle(config.turret->pitchMotor.getConfig().startAngle));
        config.pitchController->initialize();
        config.yawController->initialize();
    }

    calibrationLongTimeout.stop();
    calibrationTimer.stop();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void BalstdImuCalibrateCommand::execute()
{
    switch (calibrationState)
    {
        case CalibrationState::WAITING_FOR_SYSTEMS_ONLINE:
        {
            // Only start calibrating if the turret is online and if there is an IMU online to be
            // calibrated. The onboard Mpu6500 will never be in the `IMU_NOT_CONNECTED` state unless
            // the Mpu6500 is shorted (which has never happened). The turret MCB will only be
            // offline if the turret MCB is unplugged.
            bool turretMCBsReady = true;
            bool turretsOnline = true;

            for (auto &config : turretsAndControllers)
            {
                turretMCBsReady &= config.turretMCBCanComm->isConnected();
                turretsOnline &= config.turret->isOnline();
            }

            if (turretsOnline &&
                (turretMCBsReady ||
                 drivers->mpu6500.getImuState() != Mpu6500::ImuState::IMU_NOT_CONNECTED ||
                 drivers->chassisIsm330.getImuState() != Mpu6500::ImuState::IMU_NOT_CONNECTED))
            {
                calibrationLongTimeout.restart(MAX_CALIBRATION_WAITTIME_MS);
                calibrationTimer.restart(WAIT_TIME_TURRET_RESPONSE_MS);
                calibrationState = CalibrationState::LOCKING_TURRET;
            }

            break;
        }
        case CalibrationState::LOCKING_TURRET:
        {
            bool turretsNotMoving = true;
            for (auto &config : turretsAndControllers)
            {
                turretsNotMoving &=
                    turretReachedCenterAndNotMoving(config.turret, !config.turretImuOnPitch);
            }

            if (calibrationTimer.isExpired() && turretsNotMoving)
            {
                // enter calibration phase
                calibrationTimer.stop();

                for (auto &config : turretsAndControllers)
                {
                    config.turretMCBCanComm->requestCalibration();
                }

                drivers->mpu6500.requestCalibration();
                drivers->chassisIsm330.requestCalibration();
                calibrationState = CalibrationState::CALIBRATING_IMU;
            }

            break;
        }
        case CalibrationState::CALIBRATING_IMU:
            if (drivers->chassisIsm330.getImuState() == AbstractIMU::ImuState::IMU_CALIBRATED)
            {
                // assume turret MCB takes approximately as long as the onboard IMU to calibrate,
                // plus 1 second extra to handle sending the request and processing it
                // TODO to handle the case where the turret MCB doesn't receive information,
                // potentially add ACK sequence to turret MCB CAN comm class.
                calibrationTimer.restart(TURRET_IMU_EXTRA_WAIT_CALIBRATE_MS);
                calibrationState = CalibrationState::BUZZING;

                // TODO: config
                drivers->chassisIsm330.setAccelOffset(0.151887074f, -0.567352533f, 0.167321861f);
            }
            buzzerTimer.restart(1000);
            break;
        case CalibrationState::BUZZING:
            if (buzzerTimer.isExpired())
            {
                calibrationState = CalibrationState::WAITING_CALIBRATION_COMPLETE;
            }
            tap::buzzer::playNote(&drivers->pwm, 1000);
            break;
        case CalibrationState::WAITING_CALIBRATION_COMPLETE:
            tap::buzzer::silenceBuzzer(&drivers->pwm);
            break;
    }

    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
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

void BalstdImuCalibrateCommand::end(bool)
{
    for (auto &config : turretsAndControllers)
    {
        config.turret->yawMotor.setMotorOutput(0);
        config.turret->pitchMotor.setMotorOutput(0);
    }
}

bool BalstdImuCalibrateCommand::isFinished() const
{
    return (calibrationState == CalibrationState::WAITING_CALIBRATION_COMPLETE &&
            calibrationTimer.isExpired()) ||
           calibrationLongTimeout.isExpired();
}

}  // namespace aruwsrc::balstd
