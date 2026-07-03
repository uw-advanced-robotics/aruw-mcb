/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef DRONE_BALLISTICS_SOLVER_HPP_
#define DRONE_BALLISTICS_SOLVER_HPP_

#include "tap/util_macros.hpp"

#include "aruwsrc/algorithms/ballistics_solver_interface.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

namespace aruwsrc::control::launcher
{
class LaunchSpeedPredictorInterface;
}

namespace aruwsrc::control::turret
{
class RobotTurretSubsystem;
}

namespace aruwsrc::drone
{
class DroneTransformAdapter;

class DroneBallisticsSolver : public aruwsrc::algorithms::BallisticsSolverInterface
{
public:
    DroneBallisticsSolver(
        const aruwsrc::communication::serial::VisionCoprocessor& visionCoprocessor,
        const DroneTransformAdapter& transformAdapter,
        const control::turret::RobotTurretSubsystem& turretSubsystem,
        const control::launcher::LaunchSpeedPredictorInterface& frictionWheels,
        float defaultLaunchSpeed,
        uint8_t turretID);

    mockable std::optional<BallisticsSolution> computeTurretAimAngles() override;

private:
    const aruwsrc::communication::serial::VisionCoprocessor& visionCoprocessor;
    const DroneTransformAdapter& transformAdapter;
    const control::turret::RobotTurretSubsystem& turretSubsystem;
    const control::launcher::LaunchSpeedPredictorInterface& frictionWheels;
    const float defaultLaunchSpeed;

    uint32_t lastAimDataTimestamp = 0;
    uint32_t lastTransformTimestamp = 0;
    std::optional<BallisticsSolution> lastComputedSolution = {};
};
}  // namespace aruwsrc::drone

#endif  // DRONE_BALLISTICS_SOLVER_HPP_
