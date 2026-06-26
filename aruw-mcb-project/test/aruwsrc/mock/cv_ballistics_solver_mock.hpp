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

#ifndef CV_BALLISTICS_SOLVER_MOCK_HPP_
#define CV_BALLISTICS_SOLVER_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/algorithms/ballistics/cv_ballistics_solver.hpp"
#include "aruwsrc/algorithms/odometry/transforms/transformer_interface.hpp"

namespace aruwsrc::mock
{
namespace
{
using namespace aruwsrc::algorithms::ballistics;
}
inline constexpr CvBallisticsSolver::Config DEFAULT_CONFIG{
    .shotTimingEntryThreshold = 6.0f,
    .shotTimingExitThreshold = 4.0f,
    .defaultLaunchSpeed = 15,
    .turretPitchOffset = 0,
    .minimumShotDelay = 0.0f,
};

class CvBallisticsSolverMock : public CvBallisticsSolver
{
public:
    CvBallisticsSolverMock(
        const aruwsrc::communication::serial::VisionCoprocessor &visionCoprocessor,
        const aruwsrc::algorithms::odometry::transforms::TransformerInterface &transformer,
        const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
        CvBallisticsSolver::Config config = DEFAULT_CONFIG,
        const uint8_t turretID = 0,
        aruwsrc::communication::rtt::RttTelemetry *telemetry = nullptr);
    virtual ~CvBallisticsSolverMock();

    MOCK_METHOD(
        std::optional<CvBallisticsSolver::BallisticsSolution>,
        computeTurretAimAngles,
        (),
        (override));
};  // class CvBallisticsSolverMock
}  // namespace aruwsrc::mock

#endif  // CV_BALLISTICS_SOLVER_MOCK_HPP_
