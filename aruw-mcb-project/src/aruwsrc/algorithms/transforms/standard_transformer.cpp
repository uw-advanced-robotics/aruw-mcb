/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/algorithms/transforms/standard_transformer.hpp"
#include "tap/algorithms/transforms/transformer.hpp"

namespace aruwsrc::algorithms
{

StandardTransformer::StandardTransformer(
    tap::motor::DjiMotor& leftFrontMotor,
    tap::motor::DjiMotor& leftBackMotor,
    tap::motor::DjiMotor& rightFrontMotor,
    tap::motor::DjiMotor& rightBackMotor,
    tap::motor::DjiMotor& turretPitchMotor,
    tap::motor::DjiMotor& turretYawMotor,
    tap::communication::sensors::imu::ImuInterface& chassisImu,
    tap::communication::sensors::imu::ImuInterface& turretImu
) :
    leftFrontMotor(leftFrontMotor), leftBackMotor(leftBackMotor),
    rightFrontMotor(rightFrontMotor), rightBackMotor(rightBackMotor),
    turretPitchMotor(turretPitchMotor), turretYawMotor(turretYawMotor),
    chassisImu(chassisImu), turretImu(turretImu) 
    { };

    Transform<WorldFrame, ChassisFrame> StandardTransformer::getWorldToChassisTranform()
    {
        return worldToChassisTransform;
    }

    Transform<WorldFrame, TurretFrame> StandardTransformer::getWorldToTurretTranform()
    {
        return worldToTurretTransform;
    }

    Transform<ChassisFrame, TurretFrame> StandardTransformer::getChassisToTurretTranform()
    {
        return chassisToTurretTransform;
    }

    Transform<ChassisFrame, WorldFrame> StandardTransformer::getChassisToWorldTranform()
    {
        return chassisToWorldTransform;
    }
    
    Transform<TurretFrame, ChassisFrame> StandardTransformer::getTurretToChassisTranform()
    {
        return turretToChassisTransform;
    }

}
