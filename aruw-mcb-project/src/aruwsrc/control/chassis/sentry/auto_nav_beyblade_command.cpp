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

#include "auto_nav_beyblade_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/chassis_rel_drive.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"

using namespace tap::algorithms;
using namespace tap::communication::sensors::imu::mpu6500;
using namespace tap::communication::serial;

namespace aruwsrc
{
namespace chassis
{
AutoNavBeybladeCommand::AutoNavBeybladeCommand(
    tap::Drivers& drivers,
    HolonomicChassisSubsystem& chassis,
    const aruwsrc::control::turret::TurretMotor& yawMotor,
    aruwsrc::serial::VisionCoprocessor& visionCoprocessor,
    const tap::algorithms::odometry::Odometry2DInterface& odometryInterface,
    const aruwsrc::sentry::SentryBeybladeConfig config,
    tap::algorithms::SmoothPidConfig pidConfig,
    bool autoNavOnlyInGame)
    : drivers(drivers),
      chassis(chassis),
      yawMotor(yawMotor),
      visionCoprocessor(visionCoprocessor),
      odometryInterface(odometryInterface),
      config(config),
      autoNavOnlyInGame(autoNavOnlyInGame),
      xPid(pidConfig),
      yPid(pidConfig),
      autoNavController(chassis, visionCoprocessor.getPath(), visionCoprocessor, drivers, config)
{
    // TODO: sucks that we have to pull the address out of the reference bc everything else uses
    // pointers
    addSubsystemRequirement(&chassis);
}

// Resets ramp
void AutoNavBeybladeCommand::initialize()
{
// #ifdef ENV_UNIT_TESTS
//     rotationDirection = 1;
// #else
//     rotationDirection = (rand() - RAND_MAX / 2) < 0 ? -1 : 1;
// #endif
//     rotateSpeedRamp.reset(chassis.getDesiredRotation());
//     xRamp.reset(odometryInterface.getCurrentLocation2D().getX());
//     yRamp.reset(odometryInterface.getCurrentLocation2D().getY());
    float x = odometryInterface.getCurrentLocation2D().getX();
    float y = odometryInterface.getCurrentLocation2D().getY();
    autoNavController.initialize(Position(x, y, 0));
}

void AutoNavBeybladeCommand::execute()
{
    if (!yawMotor.isOnline()) return;

    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    prevTime = currTime;
    // Gets current chassis yaw angle
    float currentX = odometryInterface.getCurrentLocation2D().getX();
    float currentY = odometryInterface.getCurrentLocation2D().getY();
    float chassisYawAngle = odometryInterface.getYaw();

    const float maxWheelSpeed = HolonomicChassisSubsystem::getMaxWheelSpeed(
        drivers.refSerial.getRefSerialReceivingData(),
        drivers.refSerial.getRobotData().chassis.powerConsumptionLimit);

    float rampTarget = 0.0;
    
    const tap::communication::serial::RefSerialData::Rx::GameType& gametype =
        drivers.refSerial.getGameData().gameType;

    autoNavController.runController(
        currTime - prevTime,
        Position(currentX, currentY, 0),
        maxWheelSpeed,
        gametype,
        movementEnabled,
        beybladeEnabled,
        chassisYawAngle);
}

void AutoNavBeybladeCommand::end(bool) { chassis.setZeroRPM(); }
}  // namespace chassis

}  // namespace aruwsrc
