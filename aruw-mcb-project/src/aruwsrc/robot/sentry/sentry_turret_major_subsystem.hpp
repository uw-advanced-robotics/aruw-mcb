/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTRY_TURRET_MAJOR_SUBSYSTEM_HPP_
#define SENTRY_TURRET_MAJOR_SUBSYSTEM_HPP_

#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"

namespace aruwsrc::sentry  // @todo what namespace do we want? do we want to genericize this to non-pitching turrets?
{
/**
 * Turret major subsystem for the Sentry.
 *
 * Stores software necessary for interacting with a gimbal that control the
 * yaw of a turret. Provides a convenient API for other commands to interact with a turret.
 *
 * All angles computed using a right hand coordinate system. In other words, yaw is a value from
 * 0-M_TWOPI rotated counterclockwise when looking at the turret from above.
 */
// Cannot inherit RobotTurretSubsystem because it has no pitch motor
class SentryTurretMajorSubsystem final : public tap::control::Subsystem
{
public:
    SentryTurretMajorSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface* yawMotor,  // @todo stop using pointers!!
        const aruwsrc::control::turret::TurretMotorConfig& yawMotorConfig);

    void refresh();

    void initialize();

    // // Turret major has no pitch
    // inline float getWorldPitch() const override final { return 0.; };

    // // Turret major is inline with chassis center
    // inline modm::Vector3f getTurretOffset() const override final { return modm::Vector3f{0., 0., 0.}; };

    // // Turret major has no pitch lol
    // inline float getPitchOffset() const override final { return 0.; };
    /**
     * @return Timestamp of when the turret subsystem returns the angle
     * measurements.
     */
    uint32_t getLastMeasurementTimeMicroseconds() const;

    aruwsrc::control::turret::TurretMotor yawMotor;

private:
    /// Associated with and contains logic for controlling the turret's yaw motor
    uint8_t turretID;  // @todo remove?? unclear whether this helps anything or if it's just a copy-and-paste artifact

};  // class SentryTurretMajorSubsystem

}  // namespace aruwsrc::sentry

#endif  // SENTRY_TURRET_MAJOR_SUBSYSTEM_HPP_
