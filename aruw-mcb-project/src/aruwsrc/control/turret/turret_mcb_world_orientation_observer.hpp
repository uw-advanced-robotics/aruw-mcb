/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_MCB_WORLD_ORIENTATION_OBSERVER_HPP_
#define TURRET_MCB_WORLD_ORIENTATION_OBSERVER_HPP_

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"

#include "imu_world_orientation_observer.hpp"

namespace aruwsrc::control::turret
{

class TurretMcbWorldOrientationObserver
    : public ImuWorldOrientationObserver<aruwsrc::algorithms::state::Frame::TURRET>
{
public:
    TurretMcbWorldOrientationObserver(const aruwsrc::can::TurretMCBCanComm& turretMcb)
        : ImuWorldOrientationObserver(turretMcb),
          turretMcb(turretMcb)
    {
    }

    inline bool isOnline() const override { return turretMcb.isConnected(); }

private:
    const aruwsrc::can::TurretMCBCanComm& turretMcb;
};  // TurretMcbWorldOrientationObserver

}  // namespace aruwsrc::control::turret

#endif  // TURRET_MCB_WORLD_ORIENTATION_OBSERVER_HPP_