/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef ARUWSRC_COMMUNICATION_SENSORS_IMU_FUSED_IMU_HPP_
#define ARUWSRC_COMMUNICATION_SENSORS_IMU_FUSED_IMU_HPP_

#include <array>
#include <cstddef>
#include <utility>

#include "aruwsrc/algorithms/eigen_kalman_filter.hpp"
#include "tap/algorithms/transforms/dynamic_orientation.hpp"
#include "tap/algorithms/transforms/dynamic_position.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/algorithms/transforms/vector.hpp"
#include "tap/communication/sensors/imu/abstract_imu.hpp"

namespace aruwsrc::communication::sensors::imu
{
/**
 * Fuses multiple IMU sensors using a Kalman filter to produce a single, (hopefully) more accurate IMU reading.
 * 
 * @tparam N Number of IMUs to fuse
 * 
 * @note hte FusedIMU is assumed to be virtually located at the center of
 *       rotation of the rigid body on which the physical IMUs are mounted. The transform for each
 *       IMU (imuTransforms) should describe the position and orientation offset from this fusion
 *       center.
 */
template <size_t N>
class FusedImu final : public tap::communication::sensors::imu::AbstractIMU
{
public:
    FusedImu(
        const std::array<tap::communication::sensors::imu::AbstractIMU*, N>& imus,
        const std::array<tap::algorithms::transforms::Transform, N>& transforms)
        : AbstractIMU(tap::algorithms::transforms::Transform::identity()),
          imus(imus),
          imuTransforms(transforms),
          kf(makeA(), makeC(), makeQ(), makeR(), makeP0())
    {
        updateMeasurementCovariance(getImuStates());
    }

    void setImuTransform(size_t index, const tap::algorithms::transforms::Transform& transform);
    const tap::algorithms::transforms::Transform& getImuTransform(size_t index) const;

    void initialize(float sampleFrequency, float mahonyKp, float mahonyKi) override;
    void requestCalibration() override;
    void periodicIMUUpdate() override;

    inline const char* getName() const override { return "FusedIMU"; }
    inline float getAccelerationSensitivity() const override
    {
        return tap::communication::sensors::imu::GRAVITY_MPS2;
    }

private:
    static constexpr size_t kStateSize = 6;
    static constexpr size_t kMeasurementSize = N * 6;
    static constexpr size_t kPerImuMeasurementSize = 6;
    static constexpr std::array<float, kStateSize> kADiag = {1.0f, 1.0f, 1.0f,
                                                             1.0f, 1.0f, 1.0f};
    static constexpr std::array<float, kStateSize> kQDiag = {1.7e-3f, 1.3e-3f, 8.4e-3f,
                                                             2.3e-5f, 2.0e-5f, 3.9e-6f};
    static constexpr std::array<float, kStateSize> kP0Diag = {1.0f, 1.0f, 1.0f,
                                                              1.0f, 1.0f, 1.0f};

    ImuState prevImuState;

    using KalmanFilter = aruwsrc::algorithms::EigenKalmanFilter<kStateSize, kMeasurementSize>;
    using MatrixA = typename KalmanFilter::MatrixA;
    using MatrixC = typename KalmanFilter::MatrixC;
    using MatrixQ = typename KalmanFilter::MatrixQ;
    using MatrixR = typename KalmanFilter::MatrixR;
    using MatrixP = typename KalmanFilter::MatrixP;
    using VectorY = typename KalmanFilter::VectorY;

    std::array<tap::communication::sensors::imu::AbstractIMU*, N> imus;
    std::array<tap::algorithms::transforms::Transform, N> imuTransforms;

    KalmanFilter kf;
    bool kfInitialized = false;
    static constexpr float ACCEL_MEASUREMENT_VARIANCE = 1.8e-3f;
    static constexpr float GYRO_MEASUREMENT_VARIANCE = 6.6e-6f;

    void updateMeasurementCovariance(
        const std::array<tap::communication::sensors::imu::ImuInterface::ImuState, N>&
            states);

    /**
     * Transform acceleration from IMU frame to fusion frame.
     * Accounts for centripetal and Coriolis accelerations when IMU is at an offset
     * position and the robot is rotating.
     * 
     * @note Assumes the fusion frame origin is at the center of rotation of the rigid body.
     * 
     * @param fusionToImu Transform from fusion frame to IMU frame (includes position offset)
     * @param imuAcc Acceleration measured by IMU in its own frame
     * @return Acceleration in fusion frame, corrected for inertial effects
     */
    tap::algorithms::transforms::Vector transformAcceleration(
        const tap::algorithms::transforms::Transform& fusionToImu,
        const tap::algorithms::transforms::Vector& imuAcc) const;

    /**
     * Transform angular velocity from IMU frame to fusion frame.
     * Accounts for the angular velocity of the reference frame transformation.
     * 
     * @param fusionToImu Transform from fusion frame to IMU frame
     * @param imuGyro Angular velocity measured by IMU in its own frame
     * @return Angular velocity in fusion frame
     */
    tap::algorithms::transforms::Vector transformGyro(
        const tap::algorithms::transforms::Transform& fusionToImu,
        const tap::algorithms::transforms::Vector& imuGyro) const;

    static tap::communication::sensors::imu::ImuInterface::ImuState combineImuStates(
        const std::array<tap::communication::sensors::imu::ImuInterface::ImuState, N>& states);

    static inline bool isConnected(
        tap::communication::sensors::imu::ImuInterface::ImuState state)
    {
        return state !=
               tap::communication::sensors::imu::ImuInterface::ImuState::IMU_NOT_CONNECTED;
    }

    static inline bool isValid(
        tap::communication::sensors::imu::ImuInterface::ImuState state)
    {
        return state ==
                   tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATED ||
               state ==
                   tap::communication::sensors::imu::ImuInterface::ImuState::IMU_NOT_CALIBRATED;
    }

    inline std::array<tap::communication::sensors::imu::ImuInterface::ImuState, N> getImuStates()
        const
    {
        std::array<tap::communication::sensors::imu::ImuInterface::ImuState, N> states{};
        for (size_t i = 0; i < N; i++)
        {
            states[i] = imus[i]->getImuState();
        }
        return states;
    }

    template <typename MatrixT>
    static inline MatrixT makeDiagMatrix(const std::array<float, kStateSize>& diag)
    {
        MatrixT mat = MatrixT::Zero();
        for (size_t i = 0; i < kStateSize; i++)
        {
            mat(static_cast<int>(i), static_cast<int>(i)) = diag[i];
        }
        return mat;
    }

    static inline MatrixA makeA() { return makeDiagMatrix<MatrixA>(kADiag); }

    static inline MatrixC makeC()
    {
        MatrixC c = MatrixC::Zero();
        for (size_t imuIndex = 0; imuIndex < N; imuIndex++)
        {
            const size_t rowBase = imuIndex * kPerImuMeasurementSize;
            for (size_t row = 0; row < kPerImuMeasurementSize; row++)
            {
                c(static_cast<int>(rowBase + row), static_cast<int>(row)) = 1.0f;
            }
        }
        return c;
    }

    static inline MatrixQ makeQ() { return makeDiagMatrix<MatrixQ>(kQDiag); }

    static inline MatrixR makeR()
    {
        return MatrixR::Zero();
    }

    static inline MatrixP makeP0() { return makeDiagMatrix<MatrixP>(kP0Diag); }

    template <size_t... Indices>
    static inline std::array<tap::algorithms::transforms::Vector, N> makeVectorArray(
        const tap::algorithms::transforms::Vector& value,
        std::index_sequence<Indices...>)
    {
        return {((void)Indices, value)...};
    }

    static inline std::array<tap::algorithms::transforms::Vector, N> makeVectorArray(
        const tap::algorithms::transforms::Vector& value)
    {
        return makeVectorArray(value, std::make_index_sequence<N>{});
    }

};

template <size_t N>
inline void FusedImu<N>::setImuTransform(
    size_t index,
    const tap::algorithms::transforms::Transform& transform)
{
    if (index < N)
    {
        imuTransforms[index] = transform;
    }
}

template <size_t N>
inline const tap::algorithms::transforms::Transform& FusedImu<N>::getImuTransform(
    size_t index) const
{
    return imuTransforms[index];
}

template <size_t N>
inline void FusedImu<N>::initialize(float sampleFrequency, float mahonyKp, float mahonyKi)
{
    AbstractIMU::initialize(sampleFrequency, mahonyKp, mahonyKi);
    kfInitialized = false;
    requestCalibration();
}

template <size_t N>
inline void FusedImu<N>::requestCalibration()
{
    for (auto* imu : imus)
    {
        if (imu != nullptr)
        {
            imu->requestCalibration();
        }
    }

    AbstractIMU::requestCalibration();
}

template <size_t N>
inline void FusedImu<N>::periodicIMUUpdate()
{
    auto states = getImuStates();
    auto combined = combineImuStates(states);
    if (imuState == tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATING)
    {
        combined = tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATING;
    }
    imuState = combined;

    auto accel = makeVectorArray(tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f));
    auto gyro = makeVectorArray(tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f));
    std::array<bool, N> validFlags{};
    float temperatureSum = 0.0f;
    int temperatureCount = 0;
    bool anyValid = false;
    size_t firstValidIndex = N;

    for (size_t i = 0; i < N; i++)
    {
        const auto state = states[i];
        const bool connected = isConnected(state);
        const bool valid = connected && isValid(state);
        validFlags[i] = valid;
        if (valid)
        {
            tap::algorithms::transforms::Vector imuAcc(
                imus[i]->getAx(),
                imus[i]->getAy(),
                imus[i]->getAz());
            tap::algorithms::transforms::Vector imuGyro(
                imus[i]->getGx(),
                imus[i]->getGy(),
                imus[i]->getGz());
            accel[i] = transformAcceleration(imuTransforms[i], imuAcc);
            gyro[i] = transformGyro(imuTransforms[i], imuGyro);
            temperatureSum += imus[i]->getTemp();
            temperatureCount++;
            anyValid = true;
            if (firstValidIndex == N)
            {
                firstValidIndex = i;
            }
        }
        else
        {
            accel[i] = tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f);
            gyro[i] = tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f);
        }
    }

    updateMeasurementCovariance(states);

    if (!kfInitialized && anyValid)
    {
        const float initialX[kStateSize] = {
            accel[firstValidIndex].x(),
            accel[firstValidIndex].y(),
            accel[firstValidIndex].z(),
            gyro[firstValidIndex].x(),
            gyro[firstValidIndex].y(),
            gyro[firstValidIndex].z(),
        };
        kf.init(initialX);
        kfInitialized = true;
    }

    if (kfInitialized)
    {
        VectorY y;
        tap::algorithms::transforms::Vector fallbackAccel(0.0f, 0.0f, 0.0f);
        tap::algorithms::transforms::Vector fallbackGyro(0.0f, 0.0f, 0.0f);
        const auto& x = kf.getStateVectorAsMatrix();
        fallbackAccel = tap::algorithms::transforms::Vector(x[0], x[1], x[2]);
        fallbackGyro = tap::algorithms::transforms::Vector(x[3], x[4], x[5]);
        for (size_t i = 0; i < N; i++)
        {
            const size_t base = i * kPerImuMeasurementSize;
            if (!validFlags[i])
            {
                y(static_cast<int>(base + 0), 0) = fallbackAccel.x();
                y(static_cast<int>(base + 1), 0) = fallbackAccel.y();
                y(static_cast<int>(base + 2), 0) = fallbackAccel.z();
                y(static_cast<int>(base + 3), 0) = fallbackGyro.x();
                y(static_cast<int>(base + 4), 0) = fallbackGyro.y();
                y(static_cast<int>(base + 5), 0) = fallbackGyro.z();
            }
            else
            {
                y(static_cast<int>(base + 0), 0) = accel[i].x();
                y(static_cast<int>(base + 1), 0) = accel[i].y();
                y(static_cast<int>(base + 2), 0) = accel[i].z();
                y(static_cast<int>(base + 3), 0) = gyro[i].x();
                y(static_cast<int>(base + 4), 0) = gyro[i].y();
                y(static_cast<int>(base + 5), 0) = gyro[i].z();
            }
        }

        kf.performUpdate(y);
    }

    if (kfInitialized && anyValid)
    {
        // Use Kalman filter output as the fused sensor readings
        // Since we're fusing already-processed IMU data (using getAx(), getGx() etc.),
        // we output to the processed values (accG, gyroRadPerSec) not raw tick values
        const auto& x = kf.getStateVectorAsMatrix();
        imuData.accG = tap::algorithms::transforms::Vector(x[0], x[1], x[2]);
        imuData.gyroRadPerSec = tap::algorithms::transforms::Vector(x[3], x[4], x[5]);
        prevImuState = imuState;
    }
    else
    {
        // No valid IMU data available
        imuData.accG = tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f);
        imuData.gyroRadPerSec = tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f);
        prevImuState = imuState;
        imuState = ImuState::IMU_NOT_CONNECTED;
    }

    imuData.temperature = (temperatureCount > 0) ? temperatureSum / temperatureCount : 0.0f;

    AbstractIMU::periodicIMUUpdate();
}

template <size_t N>
inline void FusedImu<N>::updateMeasurementCovariance(
    const std::array<tap::communication::sensors::imu::ImuInterface::ImuState, N>& states)
{
    auto& r = kf.getMeasurementCovariance();
    
    // Initialize entire covariance matrix to zero
    for (size_t i = 0; i < r.size(); i++)
    {
        r[i] = 0.0f;
    }
    
    // Set diagonal elements for each IMU's measurements
    // R is a diagonal matrix where each IMU's measurements have independent noise
    for (size_t imuIndex = 0; imuIndex < N; imuIndex++)
    {
        // Use different variances based on IMU connection state
        const bool connected = isConnected(states[imuIndex]);
        const bool valid = connected && isValid(states[imuIndex]);
        
        // Higher variance for invalid/disconnected IMUs to reduce their influence
        const float accelVar = valid ? ACCEL_MEASUREMENT_VARIANCE : ACCEL_MEASUREMENT_VARIANCE * 100.0f;
        const float gyroVar = valid ? GYRO_MEASUREMENT_VARIANCE : GYRO_MEASUREMENT_VARIANCE * 100.0f;

        const size_t base = imuIndex * kPerImuMeasurementSize;
        
        // Set diagonal elements: R[i,i] = r[i * kMeasurementSize + i]
        r[(base + 0) * kMeasurementSize + (base + 0)] = accelVar;  // accel X
        r[(base + 1) * kMeasurementSize + (base + 1)] = accelVar;  // accel Y
        r[(base + 2) * kMeasurementSize + (base + 2)] = accelVar;  // accel Z
        r[(base + 3) * kMeasurementSize + (base + 3)] = gyroVar;   // gyro X
        r[(base + 4) * kMeasurementSize + (base + 4)] = gyroVar;   // gyro Y
        r[(base + 5) * kMeasurementSize + (base + 5)] = gyroVar;   // gyro Z
    }
}

template <size_t N>
inline tap::algorithms::transforms::Vector FusedImu<N>::transformAcceleration(
    const tap::algorithms::transforms::Transform& fusionToImu,
    const tap::algorithms::transforms::Vector& imuAcc) const
{
    // Transform chain: IMU frame -> Fusion frame -> Robot mounting frame
    // fusionToImu: fusion frame to IMU frame (includes IMU position offset and orientation)
    // imuToFusion: IMU frame to fusion frame (inverse of fusionToImu)
    // mountingTransform: fusion frame to robot mounting frame
    const auto imuToFusion = mountingTransform.compose(fusionToImu.getInverse());
    
    // For IMUs mounted at different positions on a rotating robot, we must account for:
    // 1. Centripetal acceleration: ω × (ω × r) where r is IMU position offset
    // 2. Euler acceleration: α × r (if angular acceleration present)
    // 3. Coriolis acceleration: 2ω × v (but v=0 for rigidly mounted IMU in robot frame)
    // 
    // The DynamicPosition transform handles these automatically via:
    //   af = R^T * (a - a_frame + ω × (2*(v_frame - v) + ω × (r - r_frame)))
    // 
    // Construct DynamicPosition at the IMU mounting location:
    // - position: IMU offset from fusion center (encoded in fusionToImu.translation)
    // - velocity: v = ω × r for rigidly mounted IMU
    // - acceleration: what the IMU measures in its own frame
    const auto imuPosition = fusionToImu.getTranslation();
    const auto fusionAngVel = fusionToImu.getAngularVel();
    
    // For rigidly mounted IMU: v_imu = ω_fusion × r_imu
    const auto imuVelocity = tap::algorithms::transforms::Vector(
        fusionAngVel.y() * imuPosition.z() - fusionAngVel.z() * imuPosition.y(),
        fusionAngVel.z() * imuPosition.x() - fusionAngVel.x() * imuPosition.z(),
        fusionAngVel.x() * imuPosition.y() - fusionAngVel.y() * imuPosition.x());
    
    const tap::algorithms::transforms::DynamicPosition imuDynamics(
        imuPosition.x(), imuPosition.y(), imuPosition.z(),
        imuVelocity.x(), imuVelocity.y(), imuVelocity.z(),
        imuAcc.x(), imuAcc.y(), imuAcc.z());
    
    return imuToFusion.apply(imuDynamics).getAcceleration();
}

template <size_t N>
inline tap::algorithms::transforms::Vector FusedImu<N>::transformGyro(
    const tap::algorithms::transforms::Transform& fusionToImu,
    const tap::algorithms::transforms::Vector& imuGyro) const
{
    // Transform chain: IMU frame -> Fusion frame -> Robot mounting frame
    // fusionToImu: fusion frame to IMU frame (includes IMU orientation)
    // imuToFusion: IMU frame to fusion frame (inverse of fusionToImu)
    // mountingTransform: fusion frame to robot mounting frame
    const auto imuToFusion = mountingTransform.compose(fusionToImu.getInverse());
    
    // Angular velocity must account for the rotation of reference frames:
    //   ω_out = R^T * (ω_in - ω_frame)
    // where ω_frame is the angular velocity of the frame transformation itself.
    // 
    // The DynamicOrientation transform handles this automatically.
    // Since IMU orientation is constant in robot frame, we use zero angular rate.
    const tap::algorithms::transforms::DynamicOrientation imuDynamics(
        0.0f, 0.0f, 0.0f,  // orientation (not used, only ω matters for transform)
        imuGyro.x(), imuGyro.y(), imuGyro.z());
    
    const auto fusedDynamics = imuToFusion.apply(imuDynamics);
    const auto fusedAngVel = fusedDynamics.getAngularVelocity();
    
    return tap::algorithms::transforms::Vector(
        fusedAngVel.getRollVelocity(),
        fusedAngVel.getPitchVelocity(),
        fusedAngVel.getYawVelocity());
}

template <size_t N>
inline tap::communication::sensors::imu::ImuInterface::ImuState FusedImu<N>::combineImuStates(
    const std::array<tap::communication::sensors::imu::ImuInterface::ImuState, N>& states)
{
    bool anyConnected = false;
    for (auto state : states)
    {
        if (state == tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATING)
        {
            return tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATING;
        }
        if (state == tap::communication::sensors::imu::ImuInterface::ImuState::IMU_NOT_CALIBRATED ||
            state == tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATED)
        {
            anyConnected = true;
        }
    }

    if (!anyConnected)
    {
        return tap::communication::sensors::imu::ImuInterface::ImuState::IMU_NOT_CONNECTED;
    }

    for (auto state : states)
    {
        if (state == tap::communication::sensors::imu::ImuInterface::ImuState::IMU_NOT_CALIBRATED)
        {
            return tap::communication::sensors::imu::ImuInterface::ImuState::IMU_NOT_CALIBRATED;
        }
    }

    return tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATED;
}
}  // namespace aruwsrc::communication::sensors::imu

#endif  // ARUWSRC_COMMUNICATION_SENSORS_IMU_FUSED_IMU_HPP_
