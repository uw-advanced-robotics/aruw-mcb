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

#include "tap/motor/dji_motor.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp" // change this probably
#include "tap/algorithms/transforms/transformer.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "aruwsrc/algorithms/transforms/frames.hpp"

using namespace tap::algorithms;

namespace aruwsrc::algorithms
{

/**
 * Standard-specific Transformer to handle and update world, chassis, 
 * and turret frame transforms.
 * 
 * @param leftFrontMotor Left front motor
 * @param leftBackMotor Left back motor
 * @param rightFrontMotor Right front motor
 * @param rightBackMotor Right back motor
 * @param turretPitchMotor Turret pitch motor
 * @param turretYawMotor Turret yaw motor
 * @param chassisImu Chassis IMU
 * @param turretImu Turret IMU
*/
class StandardTransformer : public Transformer
{
public:
    StandardTransformer::StandardTransformer (
        tap::motor::DjiMotor& leftFrontMotor,
        tap::motor::DjiMotor& leftBackMotor,
        tap::motor::DjiMotor& rightFrontMotor,
        tap::motor::DjiMotor& rightBackMotor,
        tap::motor::DjiMotor& turretPitchMotor,
        tap::motor::DjiMotor& turretYawMotor,
        tap::communication::sensors::imu::ImuInterface& chassisImu,
        tap::communication::sensors::imu::ImuInterface& turretImu
    );
    
    /**
     * Get World to Chassis transform
     * 
     * @returns World to Chassis transform
    */
    Transform<WorldFrame, ChassisFrame> StandardTransformer::getWorldToChassisTranform();

    /**
     * Get World to Turret transform
     * 
     * @returns World to Turret transform
    */
    Transform<WorldFrame, TurretFrame> StandardTransformer::getWorldToTurretTranform();

    /**
     * Get Chassis to Turret transform
     * 
     * @returns Chassis to Turret transform
    */
    Transform<ChassisFrame, TurretFrame> StandardTransformer::getChassisToTurretTranform();

    /**
     * Get Chassis to World transform
     * 
     * @returns Chassis to World transform
    */
    Transform<ChassisFrame, WorldFrame> StandardTransformer::getChassisToWorldTranform();
    
    /**
     * Get Turret to Chassis transform
     * 
     * @returns Turret to Chassis transform
    */
    Transform<TurretFrame, ChassisFrame> StandardTransformer::getTurretToChassisTranform();

    /**
     * Update each transform with most recent encoder and IMU odometry data. This method
     * should be called every refresh.
    */
    void StandardTransformer::update();

private:
    tap::motor::DjiMotor& leftFrontMotor;
    tap::motor::DjiMotor& leftBackMotor;
    tap::motor::DjiMotor& rightFrontMotor;
    tap::motor::DjiMotor& rightBackMotor;
    tap::motor::DjiMotor& turretPitchMotor;
    tap::motor::DjiMotor& turretYawMotor;
    tap::communication::sensors::imu::ImuInterface& chassisImu;
    tap::communication::sensors::imu::ImuInterface& turretImu;

    Transform<WorldFrame, ChassisFrame> worldToChassisTransform;
    Transform<WorldFrame, TurretFrame> worldToTurretTransform;
    Transform<ChassisFrame, TurretFrame> chassisToTurretTransform;
    Transform<ChassisFrame, WorldFrame> chassisToWorldTransform;
    Transform<TurretFrame, ChassisFrame> turretToChassisTransform;
};

}
