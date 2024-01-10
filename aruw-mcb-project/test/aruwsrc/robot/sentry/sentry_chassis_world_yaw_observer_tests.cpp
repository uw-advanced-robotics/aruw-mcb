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

#include "tap/drivers.hpp"

#include "aruwsrc/mock/turret_subsystem_mock.hpp"
#include "aruwsrc/mock/yaw_turret_subsystem_mock.hpp"
#include "aruwsrc/robot/sentry/sentry_chassis_world_yaw_observer.hpp"

using namespace testing;
using namespace aruwsrc::sentry;

class SentryChassisWorldYawObserverTest : public Test
{
protected:
    SentryChassisWorldYawObserverTest()
        : turretMajor(&drivers),
          turretMinorLeft(&drivers),
          turretMinorRight(&drivers),
          sentryWYO(turretMajor, turretMinorLeft, turretMinorRight){};
    tap::Drivers drivers;
    NiceMock<aruwsrc::mock::YawTurretSubsystemMock> turretMajor;
    NiceMock<aruwsrc::mock::TurretSubsystemMock> turretMinorLeft;
    NiceMock<aruwsrc::mock::TurretSubsystemMock> turretMinorRight;
    SentryChassisWorldYawObserver sentryWYO;

    void SetUp() override
    {
        ON_CALL(turretMajor, isOnline).WillByDefault(Return(true));
        ON_CALL(turretMinorRight.yawMotor, isOnline).WillByDefault(Return(true));
    }
};
 

//  !(turretMCB->isConnected()); issue, figure out how to fix. turretMCB?
TEST_F(SentryChassisWorldYawObserverTest, isOnline){
    float yaw;
    EXPECT_FALSE(sentryWYO.getChassisWorldYaw(&yaw));
}

