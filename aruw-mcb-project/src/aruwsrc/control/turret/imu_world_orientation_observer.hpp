#ifndef IMU_WORLD_ORIENTATION_OBSERVER_HPP_
#define IMU_WORLD_ORIENTATION_OBSERVER_HPP_

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "aruwsrc/algorithms/state/orientation_observer_interface.hpp"

namespace aruwsrc::control::turret
{

class ImuWorldOrientationObserver : public aruwsrc::algorithms::state::OrientationObserverInterface<
                                        aruwsrc::algorithms::state::Frame::WORLD,
                                        aruwsrc::algorithms::state::Frame::TURRET>
{
public:
    ImuWorldOrientationObserver(const tap::communication::sensors::imu::ImuInterface& imu);

    tap::algorithms::transforms::DynamicOrientation getOrientation() const;

    virtual bool isOnline() const override;

private:
    const tap::communication::sensors::imu::ImuInterface& imu;
};  // ImuWorldOrientationObserver

}  // namespace aruwsrc::control::turret

#endif  // IMU_WORLD_ORIENTATION_OBSERVER_HPP_