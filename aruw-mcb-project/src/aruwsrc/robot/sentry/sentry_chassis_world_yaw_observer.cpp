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

#include "sentry_chassis_world_yaw_observer.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/util_macros.hpp"
#include "modm/math/geometry/angle.hpp"

namespace aruwsrc::sentry
{
SentryChassisWorldYawObserver::SentryChassisWorldYawObserver(
    const aruwsrc::control::turret::YawTurretSubsystem& turretMajorSubsystem,
    const aruwsrc::control::turret::TurretSubsystem& turretMinorLeftSubsystem,
    const aruwsrc::control::turret::TurretSubsystem& turretMinorRightSubsystem)
    : turretMajorSubsystem(turretMajorSubsystem),
      turretMinorLeftSubsystem(turretMinorLeftSubsystem),
      turretMinorRightSubsystem(turretMinorRightSubsystem)
{
}

bool SentryChassisWorldYawObserver::getChassisWorldYaw(float* output) const 
{
    // We need both turret IMU data and turret yaw data to generate odometry which is
    // meaningful for the vision system.
    /// @todo in the future we could have the odometry subsystem fall back to using
    /// just chassis IMU and turret when turret IMU is offline.

    // the turret minor must have a turret IMU for this function to work
    // using turret minor because type C's have more accurate IMUs
    // auto turretMCB = turretMinorLeftSubsystem.getTurretMCB();
    auto turretMCB = turretMinorRightSubsystem.getTurretMCB();

    //If turretMCB is nullptr it will die in test
    #if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    #else
    assert(turretMCB != nullptr);
    #endif
    if(turretMinorRightSubsystem.yawMotor.isOnline()) return true;

    // if (!turretMCB->isConnected() || !turretMinorLeftSubsystem.yawMotor.isOnline() || !turretMajorSubsystem.yawMotor.isOnline())
    if (turretMCB != nullptr || !turretMCB->isConnected() || !turretMinorRightSubsystem.yawMotor.isOnline() || !turretMajorSubsystem.isOnline())
    {
        return false;
    }
    else
    {
        // @todo average
        // turret IMU and turret subsystem are in business

        // Spec for turretMCBCanComm doesn't say whether or not angle is normalized, so we
        // do that here. This doesn't specify which direction positive yaw sweeps.

        // TODO: for testing without chassis yaw, world yaw = majorYaw
        // so try that rather than treating Left' yaw as the world yaw

        // also, in visioncoprocessor, we send that we have 0 yaw when we are the sentry
        // so we should either change that in visioncopro, or make this (temporarily) return 0

        float turretWorldYawRadians = modm::Angle::normalize(turretMCB->getYaw());
        // turretWorldYawRadians1 = turretWorldYawRadians;
        // Normalized angle in range (-pi, pi)
        // @todo: supplant with transformer
        // float turretMinorMajorYawRadians = turretMinorLeftSubsystem.yawMotor.getAngleFromCenter();
        float turretMinorMajorYawRadians = turretMinorRightSubsystem.yawMotor.getAngleFromCenter();
        float turretMajorChassisYawRadians = turretMajorSubsystem.getChassisYaw();

        *output = modm::Angle::normalize(turretWorldYawRadians - turretMinorMajorYawRadians - turretMajorChassisYawRadians + offset);
        lastGottenYaw = *output;
        return true;
    }
}


void SentryChassisWorldYawObserver::overrideChassisYaw(float newYaw) {
    float currYaw = 0; 
    if (getChassisWorldYaw(&currYaw)) {
        // change offset
        offset += newYaw - currYaw; // normalize?
    }
}

}  // namespace aruwsrc::sentry
