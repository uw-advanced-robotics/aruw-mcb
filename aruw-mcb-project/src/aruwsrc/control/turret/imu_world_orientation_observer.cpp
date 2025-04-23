#include "imu_world_orientation_observer.hpp"

using tap::algorithms::transforms::DynamicOrientation;

namespace aruwsrc::control::turret
{

ImuWorldOrientationObserver::ImuWorldOrientationObserver(
    const tap::communication::sensors::imu::ImuInterface& imu)
    : imu(imu)
{
}

DynamicOrientation ImuWorldOrientationObserver::getOrientation() const
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
