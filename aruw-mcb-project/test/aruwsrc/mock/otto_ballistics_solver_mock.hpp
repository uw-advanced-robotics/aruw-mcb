/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef OTTO_BALLISTICS_SOLVER_MOCK_HPP_
#define OTTO_BALLISTICS_SOLVER_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"

namespace aruwsrc::mock
{
class OttoBallisticsSolverMock : public aruwsrc::algorithms::OttoBallisticsSolver
{
public:
    OttoBallisticsSolverMock(/*
        const aruwsrc::Drivers &drivers,
        const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
        const control::turret::TurretSubsystem &turretSubsystem,
        const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
        const float defaultLaunchSpeed,
        const uint8_t turretID*/);
    virtual ~OttoBallisticsSolverMock();

    MOCK_METHOD(bool, computeTurretAimAngles, (float*, float*, float*, float*), (override));
};  // class OttoBallisticsSolverMock
}  // namespace aruwsrc::mock

#endif  // OTTO_BALLISTICS_SOLVER_MOCK_HPP_
