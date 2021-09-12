#include "imu_heading_fusion.hpp"

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::algorithms
{
constexpr float ImuHeadingFusion::A[];
constexpr float ImuHeadingFusion::C[];
constexpr float ImuHeadingFusion::Q[];
constexpr float ImuHeadingFusion::R[];
constexpr float ImuHeadingFusion::P[];

ImuHeadingFusion::ImuHeadingFusion(tap::Drivers *drivers) : drivers(drivers), kf(A, C, Q, R, P) {}

void ImuHeadingFusion::initialize() { kf.init({0.0f}); }

void ImuHeadingFusion::run()
{
    if (drivers->mpu6500.initialized() && drivers->bno055InterfaceFusion.isReady())
    {
        if (initialized)
        {
            initialized = true;
            mpu6500YawOffset = drivers->mpu6500.getYaw();
        }

        inputVector.data[0] = tap::algorithms::ContiguousFloat(
            drivers->mpu6500.getYaw() - mpu6500YawOffset,
            0.0f,
            360.0f).getValue();
        inputVector.data[1] = drivers->bno055InterfaceFusion.getYaw();
        kf.performUpdate(inputVector);
    }
}
}  // namespace aruwsrc::algorithms
