#ifndef TURRET_MCB_WORLD_ORIENTATION_PROVIDER_HPP_
#define TURRET_MCB_WORLD_ORIENTATION_PROVIDER_HPP_

#include "aruwsrc/algorithms/state/orientation_provider_interface.hpp"
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"

namespace aruwsrc::control::turret
{

class TurretMcbWorldOrientationProvider
    : public aruwsrc::algorithms::state::OrientationProviderInterface<
          aruwsrc::algorithms::state::Frame::WORLD,
          aruwsrc::algorithms::state::Frame::TURRET>
{
public:
    TurretMcbWorldOrientationProvider(const aruwsrc::can::TurretMCBCanComm& turretMcb);

    tap::algorithms::transforms::DynamicOrientation getOrientation() const;

    bool providerOnline() const override;

private:
    const aruwsrc::can::TurretMCBCanComm& turretMcb;
};  // TurretMcbWorldOrientationProvider

}  // namespace aruwsrc::control::turret

#endif  // TURRET_MCB_WORLD_ORIENTATION_PROVIDER_HPP_