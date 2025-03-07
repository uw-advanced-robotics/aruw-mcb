#ifndef CHASSIS_WORLD_ORIENTATION_PROVIDER_HPP_
#define CHASSIS_WORLD_ORIENTATION_PROVIDER_HPP_

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "aruwsrc/algorithms/state/orientation_provider_interface.hpp"
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"

namespace aruwsrc::algorithms::odometry
{

template <aruwsrc::algorithms::state::Frame ADJACENT>
class ChassisWorldOrientationProvider
    : public aruwsrc::algorithms::state::OrientationProviderInterface<
          aruwsrc::algorithms::state::Frame::WORLD,
          aruwsrc::algorithms::state::Frame::CHASSIS>
{
public:
    ChassisWorldOrientationProvider(
        const tap::communication::sensors::imu::ImuInterface& chassisImu,
        const aruwsrc::algorithms::state::OrientationProviderInterface<
            aruwsrc::algorithms::state::Frame::WORLD,
            ADJACENT>& adjacentImu,
        const aruwsrc::algorithms::state::OrientationProviderInterface<
            aruwsrc::algorithms::state::Frame::CHASSIS,
            ADJACENT>& adjacentEncoder)
        : chassisImu(chassisImu),
          adjacentImu(adjacentImu),
          adjacentEncoder(adjacentEncoder)
    {
    }

    tap::algorithms::transforms::DynamicOrientation getOrientation() const;

    bool providerOnline() const override;

private:
    const tap::communication::sensors::imu::ImuInterface& chassisImu;
    const aruwsrc::algorithms::state::OrientationProviderInterface<
        aruwsrc::algorithms::state::Frame::WORLD,
        ADJACENT>& adjacentImu;
    const aruwsrc::algorithms::state::OrientationProviderInterface<
        aruwsrc::algorithms::state::Frame::CHASSIS,
        ADJACENT>& adjacentEncoder;
};  // ChassisWorldOrientationProvider

}  // namespace aruwsrc::algorithms::odometry

#endif  // CHASSIS_WORLD_ORIENTATION_PROVIDER_HPP_