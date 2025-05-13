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

    bool isOnline() const override;

private:
    const tap::communication::sensors::imu::ImuInterface& chassisImu;
    const aruwsrc::algorithms::state::OrientationObserverInterface<
        aruwsrc::algorithms::state::Frame::WORLD,
        ADJACENT>& adjacentImu;
    const aruwsrc::algorithms::state::OrientationObserverInterface<
        aruwsrc::algorithms::state::Frame::CHASSIS,
        ADJACENT>& adjacentEncoder;
};  // ChassisWorldOrientationObserver

template <aruwsrc::algorithms::state::Frame ADJACENT>
tap::algorithms::transforms::DynamicOrientation ChassisWorldOrientationObserver<
    ADJACENT>::getOrientation() const
{
    if (adjacentImu.isOnline())
    {
        return adjacentImu.getOrientation().compose(adjacentEncoder.getOrientation().inverse());
    }
    return tap::algorithms::transforms::DynamicOrientation(
        chassisImu.getRoll(),
        chassisImu.getPitch(),
        chassisImu.getYaw(),
        chassisImu.getGx(),
        chassisImu.getGy(),
        chassisImu.getGz());
}

template <aruwsrc::algorithms::state::Frame ADJACENT>
bool ChassisWorldOrientationObserver<ADJACENT>::isOnline() const
{
    // This observer handles fallback behavior, so its promise is that it will always have a valid
    // estimate.
    return true;
}

}  // namespace aruwsrc::algorithms::odometry

#endif  // CHASSIS_WORLD_ORIENTATION_OBSERVER_HPP_