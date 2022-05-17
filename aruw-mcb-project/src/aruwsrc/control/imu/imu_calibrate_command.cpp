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

#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/drivers.hpp"

using namespace tap::algorithms;
using namespace tap::communication::sensors::imu::mpu6500;

namespace aruwsrc::control::imu
{
ImuCalibrateCommand::ImuCalibrateCommand(
    aruwsrc::Drivers *drivers,
    const std::vector<std::tuple<
        aruwsrc::can::TurretMCBCanComm *,
        turret::TurretSubsystem *,
        turret::algorithms::ChassisFrameYawTurretController *,
        turret::algorithms::ChassisFramePitchTurretController *> > &turretsAndControllers,
    chassis::ChassisSubsystem *chassis,
    bool turretImuOnPitch)
    : tap::control::Command(),
      drivers(drivers),
      turretsAndControllers(turretsAndControllers),
      chassis(chassis),
      turretImuOnPitch(turretImuOnPitch)
{
    for (auto [turretMCBCanComm, turret, yawController, pitchController] : turretsAndControllers)
    {
        assert(turretMCBCanComm != nullptr);
        assert(turret != nullptr);
        assert(yawController != nullptr);
        assert(pitchController != nullptr);

        addSubsystemRequirement(turret);
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

    for (auto [turretMCBCanComm, turret, yawController, pitchController] : turretsAndControllers)
    {
        turret->yawMotor.setChassisFrameSetpoint(turret->yawMotor.getConfig().startAngle);
        turret->pitchMotor.setChassisFrameSetpoint(turret->pitchMotor.getConfig().startAngle);
        pitchController->initialize();
        yawController->initialize();
    }

    calibrationLongTimeout.stop();
    calibrationTimer.stop();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

static inline bool turretReachedCenterAndNotMoving(
    turret::TurretSubsystem *turret,
    bool ignorePitch)
{
    return compareFloatClose(
               0.0f,
               turret->yawMotor.getChassisFrameVelocity(),
               ImuCalibrateCommand::VELOCITY_ZERO_THRESHOLD) &&
           compareFloatClose(
               0.0f,
               turret->yawMotor.getAngleFromCenter(),
               ImuCalibrateCommand::POSITION_ZERO_THRESHOLD) &&
           (ignorePitch || (compareFloatClose(
                                0.0f,
                                turret->pitchMotor.getChassisFrameVelocity(),
                                ImuCalibrateCommand::VELOCITY_ZERO_THRESHOLD) &&
                            compareFloatClose(
                                0.0f,
                                turret->pitchMotor.getAngleFromCenter(),
                                ImuCalibrateCommand::POSITION_ZERO_THRESHOLD)));
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

            for (auto [turretMCBCanComm, turret, yawController, pitchController] :
                 turretsAndControllers)
            {
                turretMCBsReady &= turretMCBCanComm->isConnected();
                turretsOnline &= turret->isOnline();
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
            for (auto [turretMCBCanComm, turret, yawController, pitchController] :
                 turretsAndControllers)
            {
                turretsNotMoving &= turretReachedCenterAndNotMoving(turret, !turretImuOnPitch);
            }

            if (calibrationTimer.isExpired() && turretsNotMoving)
            {
                // enter calibration phase
                calibrationTimer.stop();

                for (auto [turretMCBCanComm, turret, yawController, pitchController] :
                     turretsAndControllers)
                {
                    turretMCBCanComm->sendImuCalibrationRequest();
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
                calibrationState = CalibrationState::WAITING_CALIBRATION_COMPLETE;
            }

            break;

        case CalibrationState::WAITING_CALIBRATION_COMPLETE:
            break;
    }

    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    // don't run pitch controller when turret IMU not on pitch (as there is no need)
    if (turretImuOnPitch)
    {
        for (auto [turretMCBCanComm, turret, yawController, pitchController] :
             turretsAndControllers)
        {
            pitchController->runController(dt, turret->pitchMotor.getChassisFrameSetpoint());
        }
    }

    for (auto [turretMCBCanComm, turret, yawController, pitchController] : turretsAndControllers)
    {
        yawController->runController(dt, turret->yawMotor.getChassisFrameSetpoint());
    }
}

void ImuCalibrateCommand::end(bool)
{
    for (auto [turretMCBCanComm, turret, yawController, pitchController] : turretsAndControllers)
    {
        turret->yawMotor.setMotorOutput(0);
        turret->pitchMotor.setMotorOutput(0);
    }
}

bool ImuCalibrateCommand::isFinished() const
{
    return (calibrationState == CalibrationState::WAITING_CALIBRATION_COMPLETE &&
            calibrationTimer.isExpired()) ||
           calibrationLongTimeout.isExpired();
}

}  // namespace aruwsrc::control::imu
