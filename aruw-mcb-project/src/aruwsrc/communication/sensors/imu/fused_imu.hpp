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

#ifndef FUSED_IMU_HPP_
#define FUSED_IMU_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "tap/algorithms/transforms/dynamic_orientation.hpp"
#include "tap/algorithms/transforms/dynamic_position.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/algorithms/transforms/vector.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/imu/abstract_imu.hpp"

#include "aruwsrc/communication/sensors/imu/fused_imu_eigen_ekf.hpp"

namespace aruwsrc::communication::sensors::imu
{
/**
 * Fuses multiple IMU sensors using a Kalman filter to produce a single, (hopefully) more accurate
 * IMU reading.
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
    /**
     * Supported IMU types for noise tuning.
     */
    enum class ImuType : uint8_t
    {
        MPU6500 = 0,
        BMI088 = 1,
        ISM330DHCX = 2,
    };

    struct Config
    {
        struct ImuNoiseDensity
        {
            std::array<float, 3> accelNoiseDensityUgSqrtHz = {0.0f, 0.0f, 0.0f};
            std::array<float, 3> gyroNoiseDensityMdpsSqrtHz = {0.0f, 0.0f, 0.0f};
        };

        ImuNoiseDensity mpu6500Noise = {
            /* accel */ {300.0f, 300.0f, 300.0f},
            /* gyro  */ {10.0f, 10.0f, 10.0f}};

        ImuNoiseDensity bmi088Noise = {
            /* accel */ {160.0f, 160.0f, 190.0f},
            /* gyro  */ {14.0f, 14.0f, 14.0f}};

        ImuNoiseDensity ism330dhcxNoise = {
            /* accel */ {100.0f, 100.0f, 100.0f},
            /* gyro  */ {8.0f, 8.0f, 8.0f}};

        /**
         * Effective bandwidth used to convert noise density -> variance.
         *
         * By default we assume BW = ODR/2 from the sampling rate used by AbstractIMU::initialize().
         *
         * To avoid exploding R when sampleFrequency is set very high (or 0), BW is clamped to:
         *   BW = clamp(0.5 * sampleFrequency, minEffectiveNoiseBandwidthHz,
         * maxEffectiveNoiseBandwidthHz).
         */
        float minEffectiveNoiseBandwidthHz = 1.0f;
        float maxEffectiveNoiseBandwidthHz = 1000.0f;

        // Higher default process noise for faster transient response in aggressive motion.
        std::array<float, 3> accelProcessVarianceRateDiag = {2.0e-1f, 2.0e-1f, 3.0e-1f};
        std::array<float, 3> gyroProcessVarianceRateDiag = {6.0e-3f, 6.0e-3f, 8.0e-3f};

        // Initial covariance P0 diagonal
        std::array<float, 6> initialStateVarianceDiag = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

        // For disconnected/invalid IMUs, inflate R to effectively ignore their measurements.
        float offlineMeasurementVarianceMultiplier = 1.0e6f;
        // Also inflate R for innovation outliers.
        float outlierVarianceMultiplier = 1.5f;
        // Innovation norm gates for outlier detection.
        float accelInnovationGate = 20.0f;  // m/s^2
        float gyroInnovationGate = 4.0f;    // rad/s
        // Clamp on adaptive R inflation due to innovations.
        float maxInnovationVarianceMultiplier = 4.0f;
    };

    /**
     * Constructor with per-IMU type selection.
     *
     * @param imus        IMU pointers
     * @param transforms  fusion->imu transforms
     * @param imuTypes    per-IMU type selector
     * @param config      tuning config
     */
    FusedImu(
        const std::array<tap::communication::sensors::imu::AbstractIMU*, N>& imus,
        const std::array<tap::algorithms::transforms::Transform, N>& transforms,
        const std::array<ImuType, N>& imuTypes,
        const Config& config = Config())
        : AbstractIMU(tap::algorithms::transforms::Transform::identity()),
          config(config),
          imus(imus),
          imuTransforms(transforms),
          imuTypes(imuTypes),
          perImuNoise(selectPerImuNoise(imuTypes, config)),
          filter(makeQ(), makeR(), makeP0())
    {
        updateProcessCovariance(1.0f);
        const auto zeroVec = makeVectorArray(tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f));
        std::array<bool, N> invalidFlags{};
        for (size_t i = 0; i < N; i++)
        {
            invalidFlags[i] = false;
        }
        updateMeasurementCovariance(getImuStates(), zeroVec, zeroVec, invalidFlags);
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

    ImuState prevImuState;

    using FilterWrapper = aruwsrc::communication::sensors::imu::FusedImuEigenEkf<N>;
    using InputVector = typename FilterWrapper::InputVector;
    using StateMatrix = typename FilterWrapper::StateMatrix;
    using InputMatrix = typename FilterWrapper::InputMatrix;

    Config config;
    std::array<tap::communication::sensors::imu::AbstractIMU*, N> imus;
    std::array<tap::algorithms::transforms::Transform, N> imuTransforms;

    // Per-IMU type and derived noise model selection.
    std::array<ImuType, N> imuTypes;
    std::array<typename Config::ImuNoiseDensity, N> perImuNoise;

    float samplePeriodS = 0.001f;
    uint32_t prevFilterUpdateTimeUs = 0;

    FilterWrapper filter;
    bool kfInitialized = false;
    bool reinitializeFilterAfterCalibration = false;

    // Helpers for per-type selection and noise->variance conversion

    static inline std::array<ImuType, N> makeDefaultImuTypes(ImuType type)
    {
        std::array<ImuType, N> types{};
        for (size_t i = 0; i < N; i++)
        {
            types[i] = type;
        }
        return types;
    }

    static inline typename Config::ImuNoiseDensity selectNoiseForType(ImuType t, const Config& cfg)
    {
        switch (t)
        {
            case ImuType::MPU6500:
                return cfg.mpu6500Noise;
            case ImuType::BMI088:
                return cfg.bmi088Noise;
            case ImuType::ISM330DHCX:
                return cfg.ism330dhcxNoise;
            default:
                return cfg.bmi088Noise;
        }
    }

    static inline std::array<typename Config::ImuNoiseDensity, N> selectPerImuNoise(
        const std::array<ImuType, N>& types,
        const Config& cfg)
    {
        std::array<typename Config::ImuNoiseDensity, N> out{};
        for (size_t i = 0; i < N; i++)
        {
            out[i] = selectNoiseForType(types[i], cfg);
        }
        return out;
    }

    inline float effectiveNoiseBandwidthHz() const
    {
        // BW = Fs/2 where Fs = 1/dt
        const float dt = (samplePeriodS > 1.0e-9f) ? samplePeriodS : 1.0e-3f;
        const float fs = 1.0f / dt;
        float bw = 0.5f * fs;
        if (bw < config.minEffectiveNoiseBandwidthHz) bw = config.minEffectiveNoiseBandwidthHz;
        if (bw > config.maxEffectiveNoiseBandwidthHz) bw = config.maxEffectiveNoiseBandwidthHz;
        return bw;
    }

    inline void measurementVarianceDiagForImu(
        size_t imuIndex,
        std::array<float, 3>& accelVarDiagOut,
        std::array<float, 3>& gyroVarDiagOut) const
    {
        const float bw = effectiveNoiseBandwidthHz();

        for (size_t a = 0; a < 3; a++)
        {
            const float nd_acc_mps2 =
                (perImuNoise[imuIndex].accelNoiseDensityUgSqrtHz[a] * 1.0e-6f) *
                tap::communication::sensors::imu::GRAVITY_MPS2;
            accelVarDiagOut[a] = (nd_acc_mps2 * nd_acc_mps2) * bw;
        }

        for (size_t a = 0; a < 3; a++)
        {
            const float nd_gyr_rad =
                modm::toRadian(perImuNoise[imuIndex].gyroNoiseDensityMdpsSqrtHz[a] * 1.0e-3f);
            gyroVarDiagOut[a] = (nd_gyr_rad * nd_gyr_rad) * bw;
        }
    }

    void updateMeasurementCovariance(
        const std::array<tap::communication::sensors::imu::ImuInterface::ImuState, N>& states,
        const std::array<tap::algorithms::transforms::Vector, N>& accel,
        const std::array<tap::algorithms::transforms::Vector, N>& gyro,
        const std::array<bool, N>& validFlags);
    void updateProcessCovariance(float dt);

    /**
     * Transform acceleration from IMU frame to fusion frame.
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

    static inline bool isConnected(tap::communication::sensors::imu::ImuInterface::ImuState state)
    {
        return state != tap::communication::sensors::imu::ImuInterface::ImuState::IMU_NOT_CONNECTED;
    }

    static inline bool isValid(tap::communication::sensors::imu::ImuInterface::ImuState state)
    {
        return state == tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATED ||
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
    static inline MatrixT makeStateDiagMatrix(const std::array<float, kStateSize>& diag)
    {
        MatrixT mat = MatrixT::Zero();
        for (size_t i = 0; i < kStateSize; i++)
        {
            mat(static_cast<int>(i), static_cast<int>(i)) = diag[i];
        }
        return mat;
    }

    inline StateMatrix makeQ()
    {
        std::array<float, kStateSize> qDiag = {
            config.accelProcessVarianceRateDiag[0],
            config.accelProcessVarianceRateDiag[1],
            config.accelProcessVarianceRateDiag[2],
            config.gyroProcessVarianceRateDiag[0],
            config.gyroProcessVarianceRateDiag[1],
            config.gyroProcessVarianceRateDiag[2]};
        return makeStateDiagMatrix<StateMatrix>(qDiag);
    }

    inline InputMatrix makeR()
    {
        InputMatrix r = InputMatrix::Zero();

        for (size_t imuIndex = 0; imuIndex < N; imuIndex++)
        {
            std::array<float, 3> accVar{};
            std::array<float, 3> gyrVar{};
            measurementVarianceDiagForImu(imuIndex, accVar, gyrVar);

            const size_t base = imuIndex * kPerImuMeasurementSize;
            r(static_cast<int>(base + 0), static_cast<int>(base + 0)) = accVar[0];
            r(static_cast<int>(base + 1), static_cast<int>(base + 1)) = accVar[1];
            r(static_cast<int>(base + 2), static_cast<int>(base + 2)) = accVar[2];
            r(static_cast<int>(base + 3), static_cast<int>(base + 3)) = gyrVar[0];
            r(static_cast<int>(base + 4), static_cast<int>(base + 4)) = gyrVar[1];
            r(static_cast<int>(base + 5), static_cast<int>(base + 5)) = gyrVar[2];
        }
        return r;
    }

    inline StateMatrix makeP0()
    {
        return makeStateDiagMatrix<StateMatrix>(config.initialStateVarianceDiag);
    }

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
    samplePeriodS = (sampleFrequency > 0.0f) ? (1.0f / sampleFrequency) : 0.001f;
    prevFilterUpdateTimeUs = tap::arch::clock::getTimeMicroseconds();

    updateProcessCovariance(samplePeriodS);
    kfInitialized = false;
    reinitializeFilterAfterCalibration = true;
    // requestCalibration();
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
    reinitializeFilterAfterCalibration = true;
}

template <size_t N>
inline void FusedImu<N>::periodicIMUUpdate()
{
    const uint32_t nowUs = tap::arch::clock::getTimeMicroseconds();
    if (prevFilterUpdateTimeUs != 0)
    {
        const uint32_t deltaUs = nowUs - prevFilterUpdateTimeUs;
        const float dynamicDt = static_cast<float>(deltaUs) * 1.0e-6f;
        if (dynamicDt > 1.0e-6f && dynamicDt < 0.1f)
        {
            samplePeriodS = dynamicDt;
        }
    }
    prevFilterUpdateTimeUs = nowUs;

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

    updateMeasurementCovariance(states, accel, gyro, validFlags);

    // Reinitialize fused EKF state/covariance on first valid sample after calibration finishes.
    if (reinitializeFilterAfterCalibration &&
        imuState != tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATING)
    {
        kfInitialized = false;
    }

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
        filter.init(initialX);
        kfInitialized = true;
        reinitializeFilterAfterCalibration = false;
    }

    if (kfInitialized)
    {
        InputVector y;
        tap::algorithms::transforms::Vector fallbackAccel(0.0f, 0.0f, 0.0f);
        tap::algorithms::transforms::Vector fallbackGyro(0.0f, 0.0f, 0.0f);
        const auto& x = filter.getStateVectorAsMatrix();
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

        updateProcessCovariance(samplePeriodS);
        (void)filter.performUpdate(y, samplePeriodS);
    }

    if (kfInitialized && anyValid)
    {
        // Use Kalman filter output as the fused sensor readings
        // Since we're fusing already-processed IMU data (using getAx(), getGx() etc.),
        // we output to the processed values (accG, gyroRadPerSec) not raw tick values
        const auto& x = filter.getStateVectorAsMatrix();
        imuData.accG = tap::algorithms::transforms::Vector(x[0], x[1], x[2]);
        imuData.gyroRadPerSec = tap::algorithms::transforms::Vector(x[3], x[4], x[5]);
        prevImuState = imuState;
    }
    else
    {
        // No valid IMU data available, reset to zero
        imuData.accG = tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f);
        imuData.gyroRadPerSec = tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f);
        prevImuState = imuState;
    }

    imuData.temperature = (temperatureCount > 0) ? temperatureSum / temperatureCount : 0.0f;

    AbstractIMU::periodicIMUUpdate();
}

template <size_t N>
inline void FusedImu<N>::updateMeasurementCovariance(
    const std::array<tap::communication::sensors::imu::ImuInterface::ImuState, N>& states,
    const std::array<tap::algorithms::transforms::Vector, N>& accel,
    const std::array<tap::algorithms::transforms::Vector, N>& gyro,
    const std::array<bool, N>& validFlags)
{
    auto& r = filter.getMeasurementCovariance();

    for (size_t i = 0; i < r.size(); i++)
    {
        r[i] = 0.0f;
    }

    const auto& x = filter.getStateVectorAsMatrix();
    const tap::algorithms::transforms::Vector predictedAccel(x[0], x[1], x[2]);
    const tap::algorithms::transforms::Vector predictedGyro(x[3], x[4], x[5]);

    for (size_t imuIndex = 0; imuIndex < N; imuIndex++)
    {
        const bool connected = isConnected(states[imuIndex]);
        const bool valid = connected && isValid(states[imuIndex]) && validFlags[imuIndex];

        float accelMultiplier = 1.0f;
        float gyroMultiplier = 1.0f;

        if (!valid)
        {
            accelMultiplier = config.offlineMeasurementVarianceMultiplier;
            gyroMultiplier = config.offlineMeasurementVarianceMultiplier;
        }
        else if (kfInitialized)
        {
            const float accelResidualNorm = std::sqrt(
                std::pow(accel[imuIndex].x() - predictedAccel.x(), 2.0f) +
                std::pow(accel[imuIndex].y() - predictedAccel.y(), 2.0f) +
                std::pow(accel[imuIndex].z() - predictedAccel.z(), 2.0f));
            const float gyroResidualNorm = std::sqrt(
                std::pow(gyro[imuIndex].x() - predictedGyro.x(), 2.0f) +
                std::pow(gyro[imuIndex].y() - predictedGyro.y(), 2.0f) +
                std::pow(gyro[imuIndex].z() - predictedGyro.z(), 2.0f));

            if (accelResidualNorm > config.accelInnovationGate)
            {
                const float ratio = accelResidualNorm / config.accelInnovationGate;
                const float adaptiveScale = ratio * ratio * config.outlierVarianceMultiplier;
                accelMultiplier = std::fmin(adaptiveScale, config.maxInnovationVarianceMultiplier);
            }
            if (gyroResidualNorm > config.gyroInnovationGate)
            {
                const float ratio = gyroResidualNorm / config.gyroInnovationGate;
                const float adaptiveScale = ratio * ratio * config.outlierVarianceMultiplier;
                gyroMultiplier = std::fmin(adaptiveScale, config.maxInnovationVarianceMultiplier);
            }
        }

        std::array<float, 3> accVar{};
        std::array<float, 3> gyrVar{};
        measurementVarianceDiagForImu(imuIndex, accVar, gyrVar);

        const size_t base = imuIndex * kPerImuMeasurementSize;
        r[(base + 0) * kMeasurementSize + (base + 0)] = accVar[0] * accelMultiplier;
        r[(base + 1) * kMeasurementSize + (base + 1)] = accVar[1] * accelMultiplier;
        r[(base + 2) * kMeasurementSize + (base + 2)] = accVar[2] * accelMultiplier;
        r[(base + 3) * kMeasurementSize + (base + 3)] = gyrVar[0] * gyroMultiplier;
        r[(base + 4) * kMeasurementSize + (base + 4)] = gyrVar[1] * gyroMultiplier;
        r[(base + 5) * kMeasurementSize + (base + 5)] = gyrVar[2] * gyroMultiplier;
    }
}

template <size_t N>
inline void FusedImu<N>::updateProcessCovariance(float dt)
{
    const float clampedDt = (dt > 1.0e-6f) ? dt : 1.0e-3f;
    auto& q = filter.getProcessCovariance();

    for (size_t i = 0; i < q.size(); i++)
    {
        q[i] = 0.0f;
    }

    q[0 * kStateSize + 0] = config.accelProcessVarianceRateDiag[0] * clampedDt;
    q[1 * kStateSize + 1] = config.accelProcessVarianceRateDiag[1] * clampedDt;
    q[2 * kStateSize + 2] = config.accelProcessVarianceRateDiag[2] * clampedDt;
    q[3 * kStateSize + 3] = config.gyroProcessVarianceRateDiag[0] * clampedDt;
    q[4 * kStateSize + 4] = config.gyroProcessVarianceRateDiag[1] * clampedDt;
    q[5 * kStateSize + 5] = config.gyroProcessVarianceRateDiag[2] * clampedDt;
}

template <size_t N>
inline tap::algorithms::transforms::Vector FusedImu<N>::transformAcceleration(
    const tap::algorithms::transforms::Transform& fusionToImu,
    const tap::algorithms::transforms::Vector& imuAcc) const
{
    const auto imuToFusion = mountingTransform.compose(fusionToImu.getInverse());

    const auto imuPosition = fusionToImu.getTranslation();
    const auto fusionAngVel = fusionToImu.getAngularVel();

    // For rigidly mounted IMU: v_imu = w_fusion x r_imu
    const auto imuVelocity = tap::algorithms::transforms::Vector(
        fusionAngVel.y() * imuPosition.z() - fusionAngVel.z() * imuPosition.y(),
        fusionAngVel.z() * imuPosition.x() - fusionAngVel.x() * imuPosition.z(),
        fusionAngVel.x() * imuPosition.y() - fusionAngVel.y() * imuPosition.x());

    const tap::algorithms::transforms::DynamicPosition imuDynamics(
        imuPosition.x(),
        imuPosition.y(),
        imuPosition.z(),
        imuVelocity.x(),
        imuVelocity.y(),
        imuVelocity.z(),
        imuAcc.x(),
        imuAcc.y(),
        imuAcc.z());

    return imuToFusion.apply(imuDynamics).getAcceleration();
}

template <size_t N>
inline tap::algorithms::transforms::Vector FusedImu<N>::transformGyro(
    const tap::algorithms::transforms::Transform& fusionToImu,
    const tap::algorithms::transforms::Vector& imuGyro) const
{
    const auto imuToFusion = mountingTransform.compose(fusionToImu.getInverse());

    const tap::algorithms::transforms::DynamicOrientation imuDynamics(
        fusionToImu.getRoll(),
        fusionToImu.getPitch(),
        fusionToImu.getYaw(),
        imuGyro.x(),
        imuGyro.y(),
        imuGyro.z());

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

#endif  // FUSED_IMU_HPP_
