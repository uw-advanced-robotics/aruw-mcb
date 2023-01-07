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
#include "tap/algorithms/kalman_filter.hpp"


using namespace tap::algorithms;

namespace aruwsrc::algorithms
{

/**
 * @param leftFrontMotor Left front motor
 * @param leftBackMotor Left back motor
 * @param rightFrontMotor Right front motor
 * @param rightBackMotor Right back motor
 * @param turretPitchMotor Turret pitch motor
 * @param turretYawMotor Turret yaw motor
 * @param chassisImu Chassis IMU
 * @param turretImu Turret IMU
 */

class StandardKFODometry {

public:
    StandardKFODometry(
        tap::motor::DjiMotor& leftFrontMotor,
        tap::motor::DjiMotor& leftBackMotor,
        tap::motor::DjiMotor& rightFrontMotor,
        tap::motor::DjiMotor& rightBackMotor,
        tap::motor::DjiMotor& turretPitchMotor,
        tap::motor::DjiMotor& turretYawMotor,
        tap::communication::sensors::imu::ImuInterface& chassisImu,
        tap::communication::sensors::imu::ImuInterface& turretImu
    );

private:

    // 
    
};

}