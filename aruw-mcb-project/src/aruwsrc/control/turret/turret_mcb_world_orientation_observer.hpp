#ifndef TURRET_MCB_WORLD_ORIENTATION_OBSERVER_HPP_
#define TURRET_MCB_WORLD_ORIENTATION_OBSERVER_HPP_

#include "aruwsrc/algorithms/state/orientation_observer_interface.hpp"
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"

namespace aruwsrc::control::turret
{

class TurretMcbWorldOrientationObserver
    : public aruwsrc::algorithms::state::OrientationObserverInterface<
          aruwsrc::algorithms::state::Frame::WORLD,
          aruwsrc::algorithms::state::Frame::TURRET>
{
public:
    TurretMcbWorldOrientationObserver(const aruwsrc::can::TurretMCBCanComm& turretMcb);

    tap::algorithms::transforms::DynamicOrientation getOrientation() const;

    bool observerOnline() const override;

private:
    const aruwsrc::can::TurretMCBCanComm& turretMcb;
};  // TurretMcbWorldOrientationObserver

}  // namespace aruwsrc::control::turret

#endif  // TURRET_MCB_WORLD_ORIENTATION_OBSERVER_HPP_