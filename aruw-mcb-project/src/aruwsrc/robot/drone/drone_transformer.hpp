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

#ifndef DRONE_TRANSFORMER_HPP_
#define DRONE_TRANSFORMER_HPP_

#include "tap/algorithms/transforms/transform.hpp"
#include "tap/communication/sensors/imu/abstract_imu.hpp"

#include "aruwsrc/control/turret/turret_orientation_interface.hpp"

namespace aruwsrc::drone
{
class DroneTransformer
{
    using Transform = tap::algorithms::transforms::Transform;

    friend class DroneTransformAdapter;

public:
    explicit DroneTransformer(
        const aruwsrc::control::turret::TurretOrientationInterface& turretOrientation,
        const tap::communication::sensors::imu::AbstractIMU* turretImu = nullptr);

    void updateTransforms();

    inline const Transform& getWorldToChassis() const { return worldToChassis; }
    inline const Transform& getWorldToTurret() const { return worldToTurret; }
    inline const Transform& getChassisToTurret() const { return chassisToTurret; }
    inline const Transform& getWorldToVTM() const { return worldToVTM; }
    inline const Transform& getChassisToArducam() const { return chassisToArducam; }

private:
    const aruwsrc::control::turret::TurretOrientationInterface& turretOrientation;
    const tap::communication::sensors::imu::AbstractIMU* turretImu;

    Transform worldToChassis;
    Transform worldToTurret;
    Transform chassisToTurret;
    Transform worldToVTM;
    Transform chassisToArducam;
};
}  // namespace aruwsrc::drone

#endif  // DRONE_TRANSFORMER_HPP_
