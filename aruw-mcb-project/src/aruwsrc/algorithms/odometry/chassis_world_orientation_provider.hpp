#ifndef CHASSIS_WORLD_ORIENTATION_OBSERVER_HPP_
#define CHASSIS_WORLD_ORIENTATION_OBSERVER_HPP_

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "aruwsrc/algorithms/state/orientation_observer_interface.hpp"
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"

namespace aruwsrc::algorithms::odometry
{

template <aruwsrc::algorithms::state::Frame ADJACENT>
class ChassisWorldOrientationObserver
    : public aruwsrc::algorithms::state::OrientationObserverInterface<
          aruwsrc::algorithms::state::Frame::WORLD,
          aruwsrc::algorithms::state::Frame::CHASSIS>
{
public:
    ChassisWorldOrientationObserver(
        const tap::communication::sensors::imu::ImuInterface& chassisImu,
        const aruwsrc::algorithms::state::OrientationObserverInterface<
            aruwsrc::algorithms::state::Frame::WORLD,
            ADJACENT>& adjacentImu,
        const aruwsrc::algorithms::state::OrientationObserverInterface<
            aruwsrc::algorithms::state::Frame::CHASSIS,
            ADJACENT>& adjacentEncoder)
        : chassisImu(chassisImu),
          adjacentImu(adjacentImu),
          adjacentEncoder(adjacentEncoder)
    {
    }

    tap::algorithms::transforms::DynamicOrientation getOrientation() const;

    bool observerOnline() const override;

private:
    const tap::communication::sensors::imu::ImuInterface& chassisImu;
    const aruwsrc::algorithms::state::OrientationObserverInterface<
        aruwsrc::algorithms::state::Frame::WORLD,
        ADJACENT>& adjacentImu;
    const aruwsrc::algorithms::state::OrientationObserverInterface<
        aruwsrc::algorithms::state::Frame::CHASSIS,
        ADJACENT>& adjacentEncoder;
};  // ChassisWorldOrientationObserver

}  // namespace aruwsrc::algorithms::odometry

#endif  // CHASSIS_WORLD_ORIENTATION_OBSERVER_HPP_