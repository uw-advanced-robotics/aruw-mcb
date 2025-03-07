#include "turret_mcb_world_orientation_provider.hpp"

using tap::algorithms::transforms::DynamicOrientation;

namespace aruwsrc::control::turret
{

TurretMcbWorldOrientationProvider::TurretMcbWorldOrientationProvider(
    const aruwsrc::can::TurretMCBCanComm& turretMcb)
    : turretMcb(turretMcb)
{
}

DynamicOrientation TurretMcbWorldOrientationProvider::getOrientation() const
{
    return DynamicOrientation(
        turretMcb.getRoll(),
        turretMcb.getPitch(),
        turretMcb.getYaw(),
        turretMcb.getRollVelocity(),
        turretMcb.getPitchVelocity(),
        turretMcb.getYawVelocity());
}

bool TurretMcbWorldOrientationProvider::providerOnline() const { return turretMcb.isConnected(); }

}  // namespace aruwsrc::control::turret
