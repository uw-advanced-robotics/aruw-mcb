/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <gtest/gtest.h>
// #include "tap/drivers.hpp"

#include "aruwsrc/algorithms/transforms/standard_transformer.hpp"
#include "modm/math/matrix.hpp"

// #include "tap"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "tap/communication/can/can_bus.hpp"


#include "tap/drivers.hpp"
#include "tap/mock/dji_motor_mock.hpp"
#include "tap/mock/mpu6500_mock.hpp"
#include "aruwsrc/mock/turret_mcb_can_comm_mock.hpp"

using namespace testing;
using namespace aruwsrc::algorithms;


class StandardTransformerTest : public Test {
protected: 
    StandardTransformerTest()
        :   chassisIMU(&drivers),
            turretMCB(&drivers, tap::can::CanBus::CAN_BUS1),
            lfM(&drivers, aruwsrc::chassis::LEFT_FRONT_MOTOR_ID, aruwsrc::chassis::CAN_BUS_MOTORS, false, "left front motor mock"),
            rfM(&drivers, aruwsrc::chassis::RIGHT_FRONT_MOTOR_ID, aruwsrc::chassis::CAN_BUS_MOTORS, false, "right front motor mock"),
            lbM(&drivers, aruwsrc::chassis::LEFT_BACK_MOTOR_ID, aruwsrc::chassis::CAN_BUS_MOTORS, false, "left back motor mock"),
            rbM(&drivers, aruwsrc::chassis::RIGHT_BACK_MOTOR_ID, aruwsrc::chassis::CAN_BUS_MOTORS, false, "right back motor mock"),
            transformer(chassisIMU, turretMCB)
    {
        transformer.init(&rfM, &lfM, &rbM, &lbM);
    }

    void SetUp() override
    {
        ON_CALL(lfM, isMotorOnline).WillByDefault(Return(true));
        ON_CALL(rfM, isMotorOnline).WillByDefault(Return(true));
        ON_CALL(lbM, isMotorOnline).WillByDefault(Return(true));
        ON_CALL(rbM, isMotorOnline).WillByDefault(Return(true));


        ON_CALL(lfM, getShaftRPM).WillByDefault(Return(100));
        ON_CALL(rfM, getShaftRPM).WillByDefault(Return(100));
        ON_CALL(lbM, getShaftRPM).WillByDefault(Return(100));
        ON_CALL(rbM, getShaftRPM).WillByDefault(Return(100));
    }

    tap::Drivers drivers;


    NiceMock<tap::mock::Mpu6500Mock> chassisIMU;
    NiceMock<aruwsrc::mock::TurretMCBCanCommMock> turretMCB;

    NiceMock<tap::mock::DjiMotorMock> lfM;
    NiceMock<tap::mock::DjiMotorMock> rfM;
    NiceMock<tap::mock::DjiMotorMock> lbM;
    NiceMock<tap::mock::DjiMotorMock> rbM;

    aruwsrc::algorithms::transforms::StandardTransformer transformer;
};


TEST_F(StandardTransformerTest, testsRun)
{
    EXPECT_TRUE(true);
}

TEST_F(StandardTransformerTest, rotateChassisVectorToWorld_whyZero)
{
    float data[3] = {10.f, 0.f, 0.f};
    modm::Matrix<float, 3, 1> chassisRelVector(data);

    transformer.rotateChassisVectorToWorld(chassisRelVector);

    EXPECT_NEAR(data[0], chassisRelVector[0][0], 1e-5);
    // std::cerr << chassisRelVector[0][0] << std::endl;
    // std::cerr << chassisRelVector[1][0] << std::endl;
    // std::cerr << chassisRelVector[2][0] << std::endl;

}