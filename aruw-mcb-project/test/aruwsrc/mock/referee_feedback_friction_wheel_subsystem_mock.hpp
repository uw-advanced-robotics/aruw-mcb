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

#ifndef REFEREE_FEEDBACK_FRICTION_WHEEL_SUBSYSTEM_MOCK_HPP_
#define REFEREE_FEEDBACK_FRICTION_WHEEL_SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::mock
{
class RefereeFeedbackFrictionWheelSubsystemMock
    : public aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<10>
{
public:
    RefereeFeedbackFrictionWheelSubsystemMock(tap::Drivers *drivers);
    virtual ~RefereeFeedbackFrictionWheelSubsystemMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, setDesiredLaunchSpeed, (float), (override));
    MOCK_METHOD(float, getDesiredLaunchSpeed, (), (const override));
    MOCK_METHOD(float, getPredictedLaunchSpeed, (), (const override));
};
}  // namespace aruwsrc::mock

#endif  // FRICTION_WHEEL_SUBSYSTEM_MOCK_HPP_
