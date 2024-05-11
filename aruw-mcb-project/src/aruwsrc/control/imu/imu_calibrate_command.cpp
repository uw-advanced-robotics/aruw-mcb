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

#include "imu_calibrate_command.hpp"

#include "tap/drivers.hpp"

#include "aruwsrc/control/turret/constants/turret_constants.hpp"

using namespace tap::algorithms;
using namespace tap::communication::sensors::imu::mpu6500;

namespace aruwsrc::control::imu
{
ImuCalibrateCommand::ImuCalibrateCommand(
    tap::Drivers *drivers,
    const std::vector<TurretIMUCalibrationConfig> &turretsAndControllers,
    chassis::HolonomicChassisSubsystem *chassis)
    : tap::control::Command(),
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

bool ImuCalibrateCommand::isReady() { return true; }

void ImuCalibrateCommand::initialize()
{
    calibrationState = CalibrationState::WAITING_FOR_SYSTEMS_ONLINE;

    if (chassis != nullptr)
    {
        chassis->setDesiredOutput(0, 0, 0);
    }

    for (auto &config : turretsAndControllers)
    {
        config.turret->yawMotor.setChassisFrameSetpoint(
            config.turret->yawMotor.getConfig().startAngle);
        config.turret->pitchMotor.setChassisFrameSetpoint(
            config.turret->pitchMotor.getConfig().startAngle);
        config.pitchController->initialize();
        config.yawController->initialize();
    }

    calibrationLongTimeout.stop();
    calibrationTimer.stop();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void ImuCalibrateCommand::execute()
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

            if (turretsOnline && (turretMCBsReady || (drivers->mpu6500.getImuState() !=
                                                      Mpu6500::ImuState::IMU_NOT_CONNECTED)))
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
                    config.turretMCBCanComm->sendImuCalibrationRequest();
                }

                drivers->mpu6500.requestCalibration();
                calibrationState = CalibrationState::CALIBRATING_IMU;
            }

            break;
        }
        case CalibrationState::CALIBRATING_IMU:
            if (drivers->mpu6500.getImuState() == Mpu6500::ImuState::IMU_CALIBRATED)
            {
                // assume turret MCB takes approximately as long as the onboard IMU to calibrate,
                // plus 1 second extra to handle sending the request and processing it
                // TODO to handle the case where the turret MCB doesn't receive information,
                // potentially add ACK sequence to turret MCB CAN comm class.
                calibrationTimer.restart(TURRET_IMU_EXTRA_WAIT_CALIBRATE_MS);
                calibrationState = CalibrationState::BUZZING;
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

void ImuCalibrateCommand::end(bool)
{
    for (auto &config : turretsAndControllers)
    {
        config.turret->yawMotor.setMotorOutput(0);
        config.turret->pitchMotor.setMotorOutput(0);
    }
}

bool ImuCalibrateCommand::isFinished() const
{
    return (calibrationState == CalibrationState::WAITING_CALIBRATION_COMPLETE &&
            calibrationTimer.isExpired()) ||
           calibrationLongTimeout.isExpired();
}

}  // namespace aruwsrc::control::imu
