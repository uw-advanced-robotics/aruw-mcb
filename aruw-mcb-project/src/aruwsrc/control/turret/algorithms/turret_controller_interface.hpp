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

#ifndef TURRET_CONTROLLER_INTERFACE_HPP_
#define TURRET_CONTROLLER_INTERFACE_HPP_

namespace aruwsrc::control::turret
{
class TurretSubsystem;

class TurretControllerInterface
{
public:
    TurretControllerInterface(TurretSubsystem *turretSubsystem) : turretSubsystem(turretSubsystem)
    {
    }

    virtual void initialize() = 0;

    virtual void runController(const uint32_t dt, const float desiredSetpoint) = 0;

    virtual float getSetpoint() const = 0;

    virtual bool isFinished() const = 0;

protected:
    TurretSubsystem *turretSubsystem;
};

class TurretPitchControllerInterface : public TurretControllerInterface
{
public:
    TurretPitchControllerInterface(TurretSubsystem *turretSubsystem)
        : TurretControllerInterface(turretSubsystem)
    {
    }
};

class TurretYawControllerInterface : public TurretControllerInterface
{
public:
    TurretYawControllerInterface(TurretSubsystem *turretSubsystem)
        : TurretControllerInterface(turretSubsystem)
    {
    }
};
}  // namespace aruwsrc::control::turret

#endif  // TURRET_CONTROLLER_INTERFACE_HPP_
