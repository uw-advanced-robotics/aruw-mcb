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

#include "otto_chassis_world_yaw_observer.hpp"

#include "tap/algorithms/wrapped_float.hpp"

#include "aruwsrc/algorithms/state/orientation_observer_interface.hpp"
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/util_macros.hpp"
#include "modm/math/geometry/angle.hpp"

using aruwsrc::algorithms::state::OrientationObserverInterface;
using tap::algorithms::WrappedFloat;

namespace aruwsrc::algorithms::odometry
{
OttoChassisWorldYawObserver::OttoChassisWorldYawObserver(
    const OrientationObserverInterface<Frame::WORLD, Frame::TURRET>& worldToTurret,
    const OrientationObserverInterface<Frame::CHASSIS, Frame::TURRET>& chassisToTurret)
    : worldToTurret(worldToTurret),
      chassisToTurret(chassisToTurret)
{
}

bool OttoChassisWorldYawObserver::getChassisWorldYaw(float* output) const
{
    // We need both turret IMU data and turret yaw data to generate odometry which is
    // meaningful for the vision system.
    /// @todo in the future we could have the odometry subsystem fall back to using
    /// just chassis IMU and turret when turret IMU is offline.

    if (!worldToTurret.isOnline() || !chassisToTurret.isOnline())
    {
        return false;
    }

    // Spec for turretMCBCanComm doesn't say whether or not angle is normalized, so we
    // do that here. This doesn't specify which direction positive yaw sweeps.
    WrappedFloat turretWorldYawRadians = Angle(worldToTurret.getOrientation().yaw());
    WrappedFloat turretChassisYawRadians = Angle(chassisToTurret.getOrientation().yaw());

    *output = (turretWorldYawRadians - turretChassisYawRadians).getWrappedValue();
    return true;
}

}  // namespace aruwsrc::algorithms::odometry
