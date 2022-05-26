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

#ifndef TURRET_CV_COMMAND_MOCK_HPP_
#define TURRET_CV_COMMAND_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"

namespace aruwsrc::mock
{
class TurretCVCommandMock : public aruwsrc::control::turret::cv::TurretCVCommand
{
public:
    TurretCVCommandMock(
        aruwsrc::Drivers *drivers,
        aruwsrc::control::turret::TurretSubsystem *turretSubsystem,
        aruwsrc::control::turret::algorithms::TurretYawControllerInterface *yawController,
        aruwsrc::control::turret::algorithms::TurretPitchControllerInterface *pitchController,
        aruwsrc::algorithms::OttoBallisticsSolver *ballisticsSolver,
        const float userPitchInputScalar,
        const float userYawInputScalar,
        uint8_t turretID = 0);
    virtual ~TurretCVCommandMock();

    MOCK_METHOD(bool, getTurretID, (), (const override));
    MOCK_METHOD(bool, isAimingWithinLaunchingTolerance, (), (const override));
};
}  // namespace aruwsrc::mock

#endif  // TURRET_CV_COMMAND_MOCK_HPP_
