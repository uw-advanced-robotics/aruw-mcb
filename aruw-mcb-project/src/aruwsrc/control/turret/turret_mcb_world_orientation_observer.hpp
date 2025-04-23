#ifndef TURRET_MCB_WORLD_ORIENTATION_OBSERVER_HPP_
#define TURRET_MCB_WORLD_ORIENTATION_OBSERVER_HPP_

#include "imu_world_orientation_observer.hpp"

namespace aruwsrc::control::turret
{

class TurretMcbWorldOrientationObserver : public ImuWorldOrientationObserver
{
public:
    inline TurretMcbWorldOrientationObserver(const aruwsrc::can::TurretMCBCanComm& turretMcb)
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