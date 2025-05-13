#ifndef IMU_WORLD_ORIENTATION_OBSERVER_HPP_
#define IMU_WORLD_ORIENTATION_OBSERVER_HPP_

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "aruwsrc/algorithms/state/orientation_observer_interface.hpp"

namespace aruwsrc::control::turret
{
template <aruwsrc::algorithms::state::Frame MOUNTING_FRAME>
class ImuWorldOrientationObserver
    : public aruwsrc::algorithms::state::
          OrientationObserverInterface<aruwsrc::algorithms::state::Frame::WORLD, MOUNTING_FRAME>
{
public:
    ImuWorldOrientationObserver(const tap::communication::sensors::imu::ImuInterface& imu);

    tap::algorithms::transforms::DynamicOrientation getOrientation() const override;

    virtual bool isOnline() const override;

private:
    const tap::communication::sensors::imu::ImuInterface& imu;
};  // ImuWorldOrientationObserver

template <Frame MOUNTING_FRAME>
ImuWorldOrientationObserver<MOUNTING_FRAME>::ImuWorldOrientationObserver(
    const tap::communication::sensors::imu::ImuInterface& imu)
    : imu(imu)
{
}

template <Frame MOUNTING_FRAME>
DynamicOrientation ImuWorldOrientationObserver<MOUNTING_FRAME>::getOrientation() const
{
    return DynamicOrientation(
        imu.getRoll(),
        imu.getPitch(),
        imu.getYaw(),
        imu.getGx(),
        imu.getGy(),
        imu.getGz());
}
}  // namespace aruwsrc::control::turret

#endif  // IMU_WORLD_ORIENTATION_OBSERVER_HPP_