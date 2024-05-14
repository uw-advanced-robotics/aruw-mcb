// /*
//  * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
//  *
//  * This file is part of aruw-mcb.
//  *
//  * aruw-mcb is free software: you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation, either version 3 of the License, or
//  * (at your option) any later version.
//  *
//  * aruw-mcb is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  * GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
//  */

// #include "otto_chassis_world_yaw_observer.hpp"

// #include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
// #include "aruwsrc/control/turret/turret_subsystem.hpp"
// #include "aruwsrc/util_macros.hpp"
// #include "modm/math/geometry/angle.hpp"

// namespace aruwsrc::chassis
// {
// OttoChassisWorldYawObserver::OttoChassisWorldYawObserver(
//     const aruwsrc::control::turret::TurretSubsystem& turretSubsystem)
//     : turretSubsystem(turretSubsystem)
// {
// }

// bool OttoChassisWorldYawObserver::getChassisWorldYaw(float* output) const
// {
//     // We need both turret IMU data and turret yaw data to generate odometry which is
//     // meaningful for the vision system.
//     /// @todo in the future we could have the odometry subsystem fall back to using
//     /// just chassis IMU and turret when turret IMU is offline.

//     // the turret must have a turret IMU for this function to work
//     auto turretMCB = turretSubsystem.getTurretMCB();
//     assert(turretMCB != nullptr);

//     if (!turretMCB->isConnected() || !turretSubsystem.yawMotor.isOnline())
//     {
//         return false;
//     }
//     else
//     {
//         // turret IMU and turret subsystem are in business

//         // Spec for turretMCBCanComm doesn't say whether or not angle is normalized, so we
//         // do that here. This doesn't specify which direction positive yaw sweeps.
//         float turretWorldYawRadians = modm::Angle::normalize(turretMCB->getYaw());
//         // Normalized angle in range (-pi, pi)
//         float turretChassisYawRadians = turretSubsystem.yawMotor.getAngleFromCenter();

//         *output = modm::Angle::normalize(turretWorldYawRadians - turretChassisYawRadians);
//         return true;
//     }
// }

// }  // namespace aruwsrc::algorithms::odometry
