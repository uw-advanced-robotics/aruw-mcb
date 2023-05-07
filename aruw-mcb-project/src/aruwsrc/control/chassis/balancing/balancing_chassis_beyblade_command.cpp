/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "balancing_chassis_beyblade_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace aruwsrc
{
namespace chassis
{
BalancingChassisBeybladeCommand::BalancingChassisBeybladeCommand(
    tap::Drivers* drivers,
    BalancingChassisSubsystem* chassis,
    control::ControlOperatorInterface& operatorInterface,
    const aruwsrc::control::turret::TurretMotor* yawMotor)
    : drivers(drivers),
      chassis(chassis),
      operatorInterface(operatorInterface),
      yawMotor(yawMotor)
{
    assert(chassis != nullptr);
    addSubsystemRequirement(chassis);
}

void BalancingChassisBeybladeCommand::initialize() { chassis->setDesiredOutput(0, 0); }

void BalancingChassisBeybladeCommand::execute()
{
    if (!chassis->getArmState()) chassis->armChassis();
    // get user input
    motionDesiredTurretRelative =
        modm::Vector2f(operatorInterface.getChassisXInput(), operatorInterface.getChassisYInput());

    // calculate pid for chassis rotation
    // returns a chassis rotation speed
    if (yawMotor->isOnline())
    {
        // Gets current turret yaw angle
        float turretYawAngle = yawMotor->getAngleFromCenter();
        const float maxWheelSpeed =
            BalancingChassisSubsystem::getMaxWheelSpeed(
                drivers->refSerial.getRefSerialReceivingData(),
                drivers->refSerial.getRobotData().chassis.powerConsumptionLimit) /
            (CHASSIS_GEARBOX_RATIO * 60 / M_TWOPI);
        // Convert from motor RPM to shaft rad/s
        // convert from wheel speed to get maximum rotation rate
        float desiredRotationRate = maxWheelSpeed * WHEEL_RADIUS / (WIDTH_BETWEEN_WHEELS_Y / 2);
        desiredRotationRate *= BEYBLADE_ROTATIONAL_SPEED_FRACTION_OF_MAX;
        // we are now turning the robot towards the desired direction.
        // Apply motion to chassis accordingly.
        float chassisXoutput = motionDesiredTurretRelative.getLength() *
                               cos(motionDesiredTurretRelative.getAngle() + turretYawAngle);
        float chassisRoutput = desiredRotationRate;

        chassis->setDesiredOutput(chassisXoutput, chassisRoutput);
        if (beybladeMode == UP_DOWN_SPIN)
        {
            if (upDownTimeout.execute())
            {
                if (chassisUp)
                {
                    chassis->setDesiredHeight(CHASSIS_HEIGHTS.first);
                    chassisUp = false;
                }
                else
                {
                    chassis->setDesiredHeight(CHASSIS_HEIGHTS.second);
                    chassisUp = true;
                }
                upDownTimeout.restart(UP_DOWN_PERIOD);
            }
        }
        else
        {
            chassisUp = false;
        }
    }
    else
    {
        // fall back to chassis drive if no turret
        chassis->setDesiredOutput(
            operatorInterface.getChassisXInput() * TRANSLATION_REMOTE_SCALAR,
            operatorInterface.getChassisYInput() * ROTATION_REMOTE_SCALAR);
    }
    chassis->setDesiredHeight(
        HEIGHT_REMOTE_SCALAR *
        drivers->remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL));
}

void BalancingChassisBeybladeCommand::end(bool interrupted)
{
    chassis->setDesiredOutput(0, 0);
    chassis->disarmChassis();
}

bool BalancingChassisBeybladeCommand::isFinished() const { return false; }
}  // namespace chassis
}  // namespace aruwsrc