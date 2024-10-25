/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/algorithms/plate_hit_tracker.hpp"

#include "tap/drivers.hpp"
#include "tap/algorithms/transforms/transform.hpp"

#include "aruwsrc/mock/transformer_interface_mock.hpp"

using namespace testing;

TEST(PlateHitTracker, no_transform_doesnt_call_ref_drivers)
{
    tap::Drivers drivers;
    aruwsrc::algorithms::PlateHitTracker hitTracker(&drivers);

    EXPECT_CALL(drivers.refSerial, getRobotData).Times(0);

    hitTracker.update();
}

TEST(PlateHitTracker, dps_less_than_last_call_never_checks_transform_after_initial_call)
{
    tap::Drivers drivers;
    tap::communication::serial::RefSerialData::Rx::RobotData data;
    data.receivedDps = 0;
    aruwsrc::algorithms::PlateHitTracker hitTracker(&drivers);

    NiceMock<aruwsrc::mock::TransformerInterfaceMock> transformer;
    hitTracker.attachTransformer(&transformer);

    tap::algorithms::transforms::Transform transform(0, 0, 0, 0, 0, 0);

    EXPECT_CALL(drivers.refSerial, getRobotData).Times(2).WillRepeatedly(ReturnRef(data));
    EXPECT_CALL(transformer, getWorldToChassis).Times(1).WillOnce(ReturnRef(transform));

    hitTracker.update();
    hitTracker.update(); // Checking this call doesn't call transformer
}

TEST(PlateHitTracker, dps_greater_than_last_call_checks_transform_after_initial_call)
{
    tap::Drivers drivers;
    tap::communication::serial::RefSerialData::Rx::RobotData data;
    data.receivedDps = 0;
    aruwsrc::algorithms::PlateHitTracker hitTracker(&drivers);

    NiceMock<aruwsrc::mock::TransformerInterfaceMock> transformer;
    hitTracker.attachTransformer(&transformer);

    tap::algorithms::transforms::Transform transform(0, 0, 0, 0, 0, 0);

    EXPECT_CALL(drivers.refSerial, getRobotData).Times(2).WillRepeatedly(ReturnRef(data));
    EXPECT_CALL(transformer, getWorldToChassis).Times(2).WillRepeatedly(ReturnRef(transform));

    hitTracker.update();
    data.receivedDps = 1;
    hitTracker.update(); // Checking this call calls transformer
}

TEST(PlateHitTracker, returns_peak_at_bin_one){
    tap::Drivers drivers;
    tap::communication::serial::RefSerialData::Rx::RobotData data;
    data.receivedDps = 1;
    data.damagedArmorId = tap::communication::serial::RefSerialData::Rx::ArmorId::FRONT;
    aruwsrc::algorithms::PlateHitTracker hitTracker(&drivers);

    NiceMock<aruwsrc::mock::TransformerInterfaceMock> transformer;
    hitTracker.attachTransformer(&transformer);

    tap::algorithms::transforms::Transform transform(0, 0, 0, 0, 0, 0);

    EXPECT_CALL(drivers.refSerial, getRobotData).Times(2).WillRepeatedly(ReturnRef(data));
    EXPECT_CALL(transformer, getWorldToChassis).Times(2).WillRepeatedly(ReturnRef(transform));

    hitTracker.update();
    data.receivedDps = 2;
    hitTracker.update();

    auto peakAngles = hitTracker.getPeakAnglesRadians();
    EXPECT_EQ(peakAngles[0].radians.getWrappedValue(), 0);
}

TEST(PlateHitTracker, detects_collision) {
    tap::Drivers drivers;
    tap::communication::serial::RefSerialData::Rx::RobotData data;
    data.receivedDps = 1;
    data.damagedArmorId = tap::communication::serial::RefSerialData::Rx::ArmorId::FRONT;
    aruwsrc::algorithms::PlateHitTracker hitTracker(&drivers);

    NiceMock<aruwsrc::mock::TransformerInterfaceMock> transformer;
    hitTracker.attachTransformer(&transformer);

    tap::algorithms::transforms::Transform transform(0, 0, 0, 0, 0, 0);

    EXPECT_CALL(drivers.refSerial, getRobotData).Times(2).WillRepeatedly(ReturnRef(data));
    EXPECT_CALL(transformer, getWorldToChassis).Times(2).WillRepeatedly(ReturnRef(transform));

    hitTracker.update();
    data.receivedDps = 2;
    data.damageType = tap::communication::serial::RefSerialData::Rx::DamageType::COLLISION;
    hitTracker.update();

    auto hitData = hitTracker.getLastHitData();
    EXPECT_EQ(hitData.projectileType, aruwsrc::algorithms::PlateHitTracker::ProjectileType::COLLISION);
}