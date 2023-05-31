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

#include "auto_nav_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/chassis_rel_drive.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"

using namespace tap::algorithms;
using namespace tap::communication::sensors::imu::mpu6500;

namespace aruwsrc
{
namespace chassis
{
AutoNavCommand::AutoNavCommand(
    tap::Drivers& drivers,
    HolonomicChassisSubsystem& chassis,
    const aruwsrc::control::turret::TurretMotor& yawMotor,
    const aruwsrc::serial::VisionCoprocessor& visionCoprocessor,
    const tap::algorithms::odometry::Odometry2DInterface& odometryInterface)
    : drivers(drivers),
      chassis(chassis),
      yawMotor(yawMotor),
      visionCoprocessor(visionCoprocessor),
      odometryInterface(odometryInterface)
{
    // TODO: sucks that we have to pull the address out of the reference bc everything else uses
    // pointers
    addSubsystemRequirement(&chassis);
}

void AutoNavCommand::initialize() {}

void AutoNavCommand::execute()
{
    // if (yawMotor.isOnline())
    if (true)
    {
        // Gets current chassis yaw angle
        float currentX = odometryInterface.getCurrentLocation2D().getX();
        float currentY = odometryInterface.getCurrentLocation2D().getY();
        float chassisYawAngle = odometryInterface.getYaw();

        const float maxWheelSpeed = HolonomicChassisSubsystem::getMaxWheelSpeed(
            drivers.refSerial.getRefSerialReceivingData(),
            drivers.refSerial.getRobotData().chassis.powerConsumptionLimit);

        aruwsrc::serial::VisionCoprocessor::AutoNavSetpointData setpointData =
            visionCoprocessor.getLastSetpointData();
        float desiredVelocityX = setpointData.x - currentX;
        float desiredVelocityY = setpointData.y - currentY;
        float mag = sqrtf(pow(desiredVelocityX, 2) + pow(desiredVelocityY, 2));
        float x = 0.0;
        float y = 0.0;
        if (mag > 0.1) {
            x = desiredVelocityX / mag * BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER * maxWheelSpeed;
            y = desiredVelocityY / mag * BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER * maxWheelSpeed;
        }

        // Rotate X and Y depending on turret angle
        tap::algorithms::rotateVector(&x, &y, -chassisYawAngle);

        // set outputs
        // TODO: i THINK this is positional offset
        chassis.setDesiredOutput(x, y, 0);

        lastX = currentX;
        lastY = currentY;
        desiredX = setpointData.x;
        desiredY = setpointData.y;
    }
}

void AutoNavCommand::end(bool) { chassis.setZeroRPM(); }
}  // namespace chassis

}  // namespace aruwsrc
