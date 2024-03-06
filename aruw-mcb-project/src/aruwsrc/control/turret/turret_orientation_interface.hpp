/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_ORIENTATION_INTERFACE_HPP_
#define TURRET_ORIENTATION_INTERFACE_HPP_

#include <stdint.h>

#include "modm/math/geometry/vector3.hpp"

namespace aruwsrc::can
{
class TurretMCBCanComm;
}

namespace aruwsrc::control::turret
{
/**
 * An interface that provides turret world yaw and pitch.
 *
 * All angles computed using a right hand coordinate system. In other words, yaw is a value from
 * 0-M_TWOPI rotated counterclockwise when looking at the turret from above. Pitch is a value from
 * 0-M_TWOPI rotated counterclockwise when looking at the turret from the right side of the turret.
 */
class TurretOrientationInterface
{
public:
    /**
     * @return An angle between [0, M_TWOPI] that is the world-relative angle of the
     * turret counterclockwise when looking at the turret from above.
     */
    virtual inline float getWorldYaw() const = 0;
    /**
     * @return An angle between [0, M_TWOPI] that is the world-relative angle of the
     * turret counterclockwise when looking at the turret from the right side.
     */
    virtual inline float getWorldPitch() const = 0;

    /**
     * @return Timestamp of when the turret subsystem returns the angle
     * measurements.
     */
    virtual inline uint32_t getLastMeasurementTimeMicros() const = 0;

    /**
     *  @return Distance between the turret and the chassis origin in the chassis frame. units of
     * meters
     */
    virtual modm::Vector3f getTurretOffset() const = 0;

    /**
     * @return Distance between the pitch axis and the yaw axis in the X-Y plane. Units meters
     */
    virtual inline float getPitchOffset() const = 0;

};  // class TurretOrientation

}  // namespace aruwsrc::control::turret

#endif  // TURRET_ORIENTATION_INTERFACE_HPP_
