/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CV_ON_TARGET_GOVERNOR_MOCK_HPP_
#define CV_ON_TARGET_GOVERNOR_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/governor/cv_on_target_governor.hpp"

namespace aruwsrc::mock
{
class CvOnTargetGovernorMock : public aruwsrc::control::governor::CvOnTargetGovernor
{
public:
    CvOnTargetGovernorMock(
        aruwsrc::Drivers &drivers,
        aruwsrc::control::turret::cv::TurretCVCommandInterface &turretCVCommand,
        aruwsrc::control::governor::AutoAimLaunchTimer &launchTimer,
        aruwsrc::control::governor::CvOnTargetGovernorMode mode);

    MOCK_METHOD(void, setGovernorEnabled, (bool), (override));
    MOCK_METHOD(bool, isGovernorEnabled, (), (const override));
    MOCK_METHOD(bool, isGovernorGating, (), (const override));
    MOCK_METHOD(bool, inShotTimingMode, (), (const override));
    MOCK_METHOD(bool, isReady, (), (override));
    MOCK_METHOD(bool, isGateSatisfied, (), (override));
    MOCK_METHOD(bool, isFinished, (), (override));
};
}  // namespace aruwsrc::mock

#endif  // CV_ON_TARGET_GOVERNOR_MOCK_HPP_
