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

#ifndef OTTO_BALLISTICS_SOLVER_MOCK_HPP_
#define OTTO_BALLISTICS_SOLVER_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"

namespace aruwsrc::mock
{
namespace
{
using namespace aruwsrc::algorithms;
}

class OttoBallisticsSolverMock : public OttoBallisticsSolver
{
public:
    OttoBallisticsSolverMock(
        const aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
        const aruwsrc::algorithms::transforms::TransformerInterface &transformer,
        const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
        float defaultLaunchSpeed,
        float turretPitchOffset,
        const tap::algorithms::transforms::Transform &worldToTurretBaseTransform,
        const aruwsrc::control::turret::TurretMotor &turretBaseMotor,
        const float turretDistFromBase,
        const uint8_t turretID);
    virtual ~OttoBallisticsSolverMock();

    MOCK_METHOD(
        std::optional<OttoBallisticsSolver::BallisticsSolution>,
        computeTurretAimAngles,
        (),
        (override));
};  // class OttoBallisticsSolverMock
}  // namespace aruwsrc::mock

#endif  // OTTO_BALLISTICS_SOLVER_MOCK_HPP_
