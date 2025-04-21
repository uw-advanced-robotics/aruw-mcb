#include "chassis_world_orientation_observer.hpp"

using aruwsrc::algorithms::state::Frame;
using tap::algorithms::transforms::DynamicOrientation;

namespace aruwsrc::algorithms::odometry
{

template <Frame ADJACENT>
DynamicOrientation ChassisWorldOrientationObserver<ADJACENT>::getOrientation() const
{
    if (adjacentImu.isOnline())
    {
        return adjacentImu.getOrientation().compose(adjacentEncoder.getOrientation().inverse());
    }
    return DynamicOrientation(
        chassisImu.getRoll(),
        chassisImu.getPitch(),
        chassisImu.getYaw(),
        chassisImu.getGx(),
        chassisImu.getGy(),
        chassisImu.getGz());
}

template <Frame ADJACENT>
bool ChassisWorldOrientationObserver<ADJACENT>::isOnline() const
{
    // This observer handles fallback behavior, so its promise is that it will always have a valid
    // estimate.
    return true;
}

}  // namespace aruwsrc::algorithms::odometry
