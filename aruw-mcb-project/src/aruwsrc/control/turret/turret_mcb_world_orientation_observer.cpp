#include "turret_mcb_world_orientation_observer.hpp"

using tap::algorithms::transforms::DynamicOrientation;

namespace aruwsrc::control::turret
{

TurretMcbWorldOrientationObserver::TurretMcbWorldOrientationObserver(
    const aruwsrc::can::TurretMCBCanComm& turretMcb)
    : turretMcb(turretMcb)
{
}

DynamicOrientation TurretMcbWorldOrientationObserver::getOrientation() const
{
    return DynamicOrientation(
        turretMcb.getRoll(),
        turretMcb.getPitch(),
        turretMcb.getYaw(),
        turretMcb.getRollVelocity(),
        turretMcb.getPitchVelocity(),
        turretMcb.getYawVelocity());
}

bool TurretMcbWorldOrientationObserver::isOnline() const { return turretMcb.isConnected(); }

}  // namespace aruwsrc::control::turret
