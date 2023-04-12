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

namespace aruwsrc::control::turret
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
class SentryTurretMajorSubsystem final : public tap::control::Subsystem
{
public:
    SentryTurretMajorSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface* yawMotor,
        const TurretMotorConfig& yawMotorConfig,
        uint8_t turretID);

    /**
     * @return An angle between [0, M_TWOPI] that is the world-relative angle of the
     * turret counterclockwise when looking at the turret from above.
     */
    float getWorldYaw() const;
    /**
     * @return Timestamp of when the turret subsystem returns the angle
     * measurements.
     */
    uint32_t getLastMeasurementTimeMicroseconds() const;
    /**
     *  @return Distance between the turret and the chassis origin in the chassis frame. units of
     * meters
     */
    modm::Vector3f getTurretOffset() const;

private:
    /// Associated with and contains logic for controlling the turret's yaw motor
    TurretMotor yawMotor;
    uint8_t turretID;

};  // class SentryTurretMajorSubsystem

}  // namespace aruwsrc::control::turret

#endif  // SENTRY_TURRET_MAJOR_SUBSYSTEM_HPP_
