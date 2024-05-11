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
// @todo: this sentry-specific imu calibrate command adds odometry reset functionality, which
// should be incorporated into the regular imu calibrate command. After incorporating, this class
// can be deleted
#include "sentry_imu_calibrate_command.hpp"

#include "tap/drivers.hpp"

#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"

using namespace tap::algorithms;
using namespace tap::communication::sensors::imu::mpu6500;

namespace aruwsrc::control::imu
{
// TODO: we want to be able to calibrate an arbitrary turret subsystem (one that
// has pitch OR yaw OR both)
SentryImuCalibrateCommand::SentryImuCalibrateCommand(
    tap::Drivers *drivers,
    const std::vector<TurretIMUCalibrationConfig> &turretsAndControllers,
    aruwsrc::control::turret::YawTurretSubsystem &turretMajor,
    aruwsrc::control::turret::algorithms::TurretYawControllerInterface &turretMajorController,
    chassis::HolonomicChassisSubsystem &chassis,
    aruwsrc::sentry::SentryChassisWorldYawObserver &yawObserver,
    aruwsrc::sentry::SentryKFOdometry2DSubsystem &odometryInterface,
    aruwsrc::virtualMCB::MCBLite &majorMCBLite,
    aruwsrc::virtualMCB::MCBLite &chassisMCBLite)
    : imu::ImuCalibrateCommand(drivers, turretsAndControllers, &chassis),
      turretMajor(turretMajor),
      turretMajorController(turretMajorController),
      yawObserver(yawObserver),
      odometryInterface(odometryInterface),
      majorMCBLite(majorMCBLite),
      chassisMCBLite(chassisMCBLite)
{
    addSubsystemRequirement(&turretMajor);
}

void SentryImuCalibrateCommand::initialize()
{
    yawObserver.overrideChassisYaw(0);
    // reset odometry
    odometryInterface.reset();

    ImuCalibrateCommand::initialize();

    // initialize major
    turretMajor.getMutableMotor().setChassisFrameSetpoint(
        turretMajor.getReadOnlyMotor()
            .getConfig()
            .startAngle);  // @todo really sus interdependency with imu
                           // drift because assumes world controller
    turretMajorController.initialize();

    calibrationLongTimeout.stop();
    calibrationTimer.stop();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

static inline bool turretMajorReachedCenterAndNotMoving(turret::YawTurretSubsystem &turret)
{
    // return true;
    return compareFloatClose(
               0.0f,
               turret.getReadOnlyMotor().getChassisFrameVelocity(),
               SentryImuCalibrateCommand::VELOCITY_ZERO_THRESHOLD) &&
           compareFloatClose(
               0.0f,
               turret.getReadOnlyMotor().getAngleFromCenter(),
               SentryImuCalibrateCommand::POSITION_ZERO_THRESHOLD);
}

size_t i;
void SentryImuCalibrateCommand::execute()
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

            turretsNotMoving &= turretMajorReachedCenterAndNotMoving(turretMajor);

            if (calibrationTimer.isExpired() && turretsNotMoving)
            {
                // enter calibration phase
                calibrationTimer.stop();

                for (auto &config : turretsAndControllers)
                {
                    config.turretMCBCanComm->sendImuCalibrationRequest();
                }

                drivers->mpu6500.requestCalibration();

                chassisMCBLite.imu.requestCalibration();
                majorMCBLite.imu.requestCalibration();

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

    turretMajorController.runController(
        dt,
        turretMajor.getReadOnlyMotor().getChassisFrameSetpoint() + odometryInterface.getYaw());
}

void SentryImuCalibrateCommand::end(bool)
{
    // TODO: this being commented out causes turrets to hold position when this deschedules
    // change if you want
    // for (auto &config : turretsAndControllers)
    // {
    //     config.turret->yawMotor.setMotorOutput(0);
    //     config.turret->pitchMotor.setMotorOutput(0);
    // }

    // turretMajor->yawMotor.setMotorOutput(0);
}

}  // namespace aruwsrc::control::imu
