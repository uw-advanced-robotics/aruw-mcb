/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

/*
 * Copyright (c) 2019 Sanger_X
 */

#include "swerve_module.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "aruwsrc/drivers.hpp"


using namespace tap::algorithms;

namespace aruwsrc
{
namespace chassis
{

SwerveModule::SwerveModule(
    aruwsrc::Drivers* drivers,
    tap::motor::MotorId driveMotorId,
    tap::motor::MotorId azimuthMotorId,
    SwerveModuleConfig& swerveModuleConfig)
    : driveMotor(
          drivers,
          driveMotorId,
          CAN_BUS_MOTORS,
          swerveModuleConfig.driveMotorInverted,
          "Drive motor"),
      azimuthMotor(
          drivers,
          azimuthMotorId,
          CAN_BUS_MOTORS,
          swerveModuleConfig.azimuthMotorInverted,
          "Azimuth motor"),
      drivePid(
          swerveModuleConfig.drivePidKp,
          swerveModuleConfig.drivePidKi,
          swerveModuleConfig.drivePidKd,
          swerveModuleConfig.drivePidMaxIntergalErrorSum,
          swerveModuleConfig.drivePidMaxOutput),
      azimuthPid(
          swerveModuleConfig.azimuthPidKp,
          swerveModuleConfig.azimuthPidKi,
          swerveModuleConfig.azimuthPidKd,
          swerveModuleConfig.azimuthPidMaxIntergalErrorSum,
          swerveModuleConfig.azimuthPidMaxOutput)
{
}

void SwerveModule::setDesiredState(float metersPerSecond, float radianOutput) {}

float SwerveModule::getVelocity() {}

float SwerveModule::getAngle() {}

}  // namespace chassis
}  // namespace aruwsrc