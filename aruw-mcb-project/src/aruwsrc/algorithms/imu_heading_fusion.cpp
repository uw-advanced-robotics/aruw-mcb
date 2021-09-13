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


float A_a[ImuHeadingFusion::KF_STATES * ImuHeadingFusion::KF_STATES] = {1};
float C_a[ImuHeadingFusion::KF_INPUTS * ImuHeadingFusion::KF_STATES] = {1, 1};

float Q_a[ImuHeadingFusion::KF_STATES * ImuHeadingFusion::KF_STATES] = {1};
float R_a[ImuHeadingFusion::KF_INPUTS * ImuHeadingFusion::KF_INPUTS] = {1, 1};

float P_a[ImuHeadingFusion::KF_STATES * ImuHeadingFusion::KF_STATES] = {1};

ImuHeadingFusion::ImuHeadingFusion(tap::Drivers *drivers) : drivers(drivers), kf(A_a, C_a, Q_a, R_a, P_a) {}

void ImuHeadingFusion::initialize() { kf.init({0.0f}); }

void ImuHeadingFusion::run()
{
    if (drivers->mpu6500.initialized() && drivers->bno055InterfaceFusion.isReady())
    {
        if (!initialized)
        {
            initialized = true;
            mpu6500YawOffset = drivers->mpu6500.getYaw();
        }

        inputVector.data[0] = tap::algorithms::ContiguousFloat(mpu6500YawOffset - drivers->mpu6500.getYaw(), 0, 360).getValue();
        inputVector.data[1] = drivers->bno055InterfaceFusion.getYaw();

        drivers->terminalSerial.getStream().printf(
            "%li\t%.2f\t%.2f\n",
            tap::arch::clock::getTimeMilliseconds(),
            kf.getStateMatrix()[0] / 2.0f,
            // drivers->imuHeadingFusion.getInputVector()[0],
            drivers->imuHeadingFusion.getInputVector()[1]);

        kf.performUpdate(inputVector);
    }
}
}  // namespace aruwsrc::algorithms
