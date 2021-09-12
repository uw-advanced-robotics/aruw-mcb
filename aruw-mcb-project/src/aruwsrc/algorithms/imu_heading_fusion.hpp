#ifndef IMU_HEADING_FUSION_HPP_
#define IMU_HEADING_FUSION_HPP_

#include "kalman_filter.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::algorithms
{
class ImuHeadingFusion
{
public:
    static constexpr uint16_t KF_STATES = 1;
    static constexpr uint16_t KF_INPUTS = 2;

    ImuHeadingFusion(tap::Drivers *drivers);

    void initialize();

    void run();

    float getYaw() const { return kf.getStateMatrix()[0]; }

private:
    static constexpr float A [KF_STATES * KF_STATES] = {1};
    static constexpr float C [KF_INPUTS * KF_STATES] = {1,1};
    static constexpr float Q [KF_STATES * KF_STATES] = {1};
    static constexpr float R [KF_INPUTS * KF_INPUTS] = {1,1};
    static constexpr float P [KF_STATES * KF_STATES] = {1};

    tap::Drivers *drivers;

    CMSISMat<KF_INPUTS, 1> inputVector;

    KalmanFilter<KF_STATES, KF_INPUTS> kf;

    bool initialized = false;

    float mpu6500YawOffset = 0.0f;
};
}  // namespace aruwsrc::algorithms

#endif  // IMU_HEADING_FUSION_HPP_
