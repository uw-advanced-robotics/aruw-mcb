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

#ifndef FUSED_IMU_MEKF_KF_HPP_
#define FUSED_IMU_MEKF_KF_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "tap/algorithms/transforms/dynamic_orientation.hpp"
#include "tap/algorithms/transforms/dynamic_position.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/algorithms/transforms/vector.hpp"
#include "tap/algorithms/wrapped_float.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/imu/abstract_imu.hpp"

#include "aruwsrc/communication/rtt/rtt_telemetry.hpp"
#include "aruwsrc/communication/sensors/imu/fused_imu_eigen_ekf.hpp"

namespace aruwsrc::communication::sensors::imu
{
template <size_t N>
class FusedImuMekfKf final : public tap::communication::sensors::imu::AbstractIMU
{
public:
    static_assert(N > 0, "FusedImuMekfKf requires at least one IMU");

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

        float minEffectiveNoiseBandwidthHz = 1.0f;
        float maxEffectiveNoiseBandwidthHz = 1000.0f;

        float accelGateMps2 = 4.0f;
        float minAccelNormMps2 = 4.0f;
        float maxAccelNormMps2 = 15.0f;

        float gyroBiasRandomWalkStdRadPerSec = 5.0e-5f;
        float accelBiasRandomWalkStdMps2 = 1.0e-3f;
        float initialAngleStdRad = 0.2f;
        float initialGyroBiasStdRadPerSec = 0.1f;
        float initialAccelBiasStdMps2 = 0.5f;

        // Linear KF parameters used to fuse accel/gyro measurements from all IMUs.
        std::array<float, 3> accelProcessVarianceRateDiag = {2.0e-1f, 2.0e-1f, 3.0e-1f};
        std::array<float, 3> gyroProcessVarianceRateDiag = {6.0e-3f, 6.0e-3f, 8.0e-3f};
        std::array<float, 6> initialSignalStateVarianceDiag = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
        float offlineMeasurementVarianceMultiplier = 1.0e6f;
        float outlierVarianceMultiplier = 1.5f;
        float accelInnovationGate = 20.0f;
        float gyroInnovationGate = 4.0f;
        float maxInnovationVarianceMultiplier = 4.0f;
        float minMeasurementVariance = 1.0e-8f;
        float maxMeasurementVariance = 1.0e8f;

        float accelMeasurementVarianceScale = 8.0f;
        float accelDynamicVarianceGain = 2.0f;
        float accelDynamicVarianceMaxScale = 40.0f;
        float accelNisGate = 16.0f;
        float attitudeCorrectionGain = 0.10f;
        float gyroBiasCorrectionGain = 1.0e-6f;
        float accelBiasCorrectionGain = 1.0e-7f;
        float maxGyroBiasAbsRadPerSec = 2.0e-2f;
        float maxAccelBiasAbsMps2 = 1.0e-1f;
        bool suppressYawCorrectionFromAccel = true;
    };

    FusedImuMekfKf(
        const std::array<tap::communication::sensors::imu::AbstractIMU*, N>& imus,
        const std::array<tap::algorithms::transforms::Transform, N>& transforms,
        const std::array<ImuType, N>& imuTypes,
        const Config& config = Config(),
        aruwsrc::communication::rtt::RttTelemetry* telemetry = nullptr)
        : AbstractIMU(tap::algorithms::transforms::Transform::identity()),
          config(config),
          imus(imus),
          imuTransforms(transforms),
          imuToFusionTransforms(transforms),
          perImuNoise(selectPerImuNoise(imuTypes, config)),
          signalFilter(makeSignalQ(), makeSignalR(), makeSignalP0()),
          telemetry(telemetry)
    {
        accelInnovationGateSq = config.accelInnovationGate * config.accelInnovationGate;
        gyroInnovationGateSq = config.gyroInnovationGate * config.gyroInnovationGate;
        this->recomputeImuToFusionTransforms();
        resetFilterState();
    }

    void initialize(float sampleFrequency, float mahonyKp, float mahonyKi) override
    {
        (void)mahonyKp;
        (void)mahonyKi;
        AbstractIMU::initialize(sampleFrequency, 0.0f, 0.0f);
        samplePeriodS = (sampleFrequency > 0.0f) ? (1.0f / sampleFrequency) : 0.001f;
        prevFilterUpdateTimeUs = tap::arch::clock::getTimeMicroseconds();
        signalFilterInitialized = false;
        filterInitialized = false;
        pendingReinitializeAfterCalibration = true;
        resetFilterState();
        // requestCalibration();
    }

    void setCalibrationSamples(int sampleCount)
    {
        for (auto* imu : imus)
        {
            if (imu != nullptr)
            {
                imu->setCalibrationSamples(sampleCount);
            }
        }
        offsetSampleCount = sampleCount;
    }

    void requestCalibration() override
    {
        signalFilterInitialized = false;
        filterInitialized = false;
        pendingReinitializeAfterCalibration = true;
        resetFilterState();
        for (auto* imu : imus)
        {
            if (imu != nullptr)
            {
                imu->requestCalibration();
            }
        }
        imuState = tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATING;
    }

    void setImuTransform(size_t index, const tap::algorithms::transforms::Transform& transform)
    {
        if (index < N)
        {
            imuTransforms[index] = transform;
            this->recomputeImuToFusionTransform(index);
        }
    }

    void setFusionMountingTransform(const tap::algorithms::transforms::Transform& transform)
    {
        AbstractIMU::setMountingTransform(transform);
        this->recomputeImuToFusionTransforms();
    }

    const tap::algorithms::transforms::Transform& getImuTransform(size_t index) const
    {
        return imuTransforms[index];
    }

    void periodicIMUUpdate() override
    {
        const uint32_t cycleStartUs = tap::arch::clock::getTimeMicroseconds();
        const auto logCycleTime = [&](const char* label) {
            if (telemetry != nullptr)
            {
                telemetry->logSignal(
                    label,
                    tap::arch::clock::getTimeMicroseconds() - cycleStartUs);
            }
        };

        if (prevFilterUpdateTimeUs != 0U)
        {
            const uint32_t deltaUs = cycleStartUs - prevFilterUpdateTimeUs;
            const float dtMeasured = static_cast<float>(deltaUs) * 1.0e-6f;
            if (dtMeasured > 1.0e-6f && dtMeasured < 0.05f)
            {
                samplePeriodS = dtMeasured;
            }
        }
        prevFilterUpdateTimeUs = cycleStartUs;

        const auto states = getImuStates();
        imuState = combineImuStates(states);
        if (imuState == tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATING)
        {
            pendingReinitializeAfterCalibration = true;
            signalFilterInitialized = false;
            filterInitialized = false;
            imuData.accG = tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f);
            imuData.gyroRadPerSec = tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f);
            imuData.temperature = 0.0f;
            logCycleTime("perf/fused_imu/periodic_total_us");
            return;
        }

        auto accel = makeVectorArray(tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f));
        auto gyro = makeVectorArray(tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f));
        std::array<bool, N> validFlags{};
        bool anyValid = false;
        float tempSum = 0.0f;
        int tempCount = 0;
        size_t firstValidIndex = N;

        for (size_t i = 0; i < N; i++)
        {
            const bool valid = isConnected(states[i]) && isValid(states[i]);
            validFlags[i] = valid;
            if (!valid)
            {
                continue;
            }

            const tap::algorithms::transforms::Vector imuAcc(
                imus[i]->getAx(),
                imus[i]->getAy(),
                imus[i]->getAz());
            const tap::algorithms::transforms::Vector imuGyro(
                imus[i]->getGx(),
                imus[i]->getGy(),
                imus[i]->getGz());

            accel[i] = transformAcceleration(imuTransforms[i], imuToFusionTransforms[i], imuAcc);
            gyro[i] = transformGyro(imuTransforms[i], imuToFusionTransforms[i], imuGyro);
            tempSum += imus[i]->getTemp();
            tempCount++;
            anyValid = true;
            if (firstValidIndex == N)
            {
                firstValidIndex = i;
            }
        }

        if (!anyValid)
        {
            imuData.accG = tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f);
            imuData.gyroRadPerSec = tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f);
            imuData.temperature = 0.0f;
            logCycleTime("perf/fused_imu/periodic_total_us");
            return;
        }

        if (pendingReinitializeAfterCalibration &&
            imuState != tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATING)
        {
            signalFilterInitialized = false;
            filterInitialized = false;
        }

        updatePerImuNoiseVariance();
        updateFusedMeasurementVariance(validFlags);

        if (!signalFilterInitialized && firstValidIndex != N)
        {
            const float initialX[signalStateSize] = {
                accel[firstValidIndex].x(),
                accel[firstValidIndex].y(),
                accel[firstValidIndex].z(),
                gyro[firstValidIndex].x(),
                gyro[firstValidIndex].y(),
                gyro[firstValidIndex].z()};
            signalFilter.init(initialX);
            signalFilterInitialized = true;
        }

        updateSignalMeasurementCovariance(states, accel, gyro, validFlags);

        const uint32_t signalFilterStartUs = tap::arch::clock::getTimeMicroseconds();
        if (signalFilterInitialized)
        {
            updateSignalProcessCovariance(samplePeriodS);
            if (signalFilter.predict(samplePeriodS) == 0)
            {
                const auto& xPred = signalFilter.getStateVectorAsMatrix();
                SignalInputVector zPredicted;
                setVectorElem(zPredicted, 0, xPred[0]);
                setVectorElem(zPredicted, 1, xPred[1]);
                setVectorElem(zPredicted, 2, xPred[2]);
                setVectorElem(zPredicted, 3, xPred[3]);
                setVectorElem(zPredicted, 4, xPred[4]);
                setVectorElem(zPredicted, 5, xPred[5]);
                for (size_t i = 0; i < N; i++)
                {
                    if (!validFlags[i])
                    {
                        (void)signalFilter.updateSingleImu(static_cast<uint16_t>(i), zPredicted);
                    }
                    else
                    {
                        SignalInputVector zBlock;
                        setVectorElem(zBlock, 0, accel[i].x());
                        setVectorElem(zBlock, 1, accel[i].y());
                        setVectorElem(zBlock, 2, accel[i].z());
                        setVectorElem(zBlock, 3, gyro[i].x());
                        setVectorElem(zBlock, 4, gyro[i].y());
                        setVectorElem(zBlock, 5, gyro[i].z());
                        (void)signalFilter.updateSingleImu(static_cast<uint16_t>(i), zBlock);
                    }
                }
            }
        }
        if (telemetry != nullptr)
        {
            telemetry->logSignal(
                "perf/fused_imu/signal_filter_us",
                tap::arch::clock::getTimeMicroseconds() - signalFilterStartUs);
        }

        tap::algorithms::transforms::Vector fusedGyro(0.0f, 0.0f, 0.0f);
        tap::algorithms::transforms::Vector fusedAccel(0.0f, 0.0f, 0.0f);
        if (signalFilterInitialized)
        {
            const auto& xSignal = signalFilter.getStateVectorAsMatrix();
            fusedAccel = tap::algorithms::transforms::Vector(xSignal[0], xSignal[1], xSignal[2]);
            fusedGyro = tap::algorithms::transforms::Vector(xSignal[3], xSignal[4], xSignal[5]);
        }
        else if (firstValidIndex != N)
        {
            fusedAccel = accel[firstValidIndex];
            fusedGyro = gyro[firstValidIndex];
        }

        if (!filterInitialized)
        {
            gyroBias = {0.0f, 0.0f, 0.0f};
            accelBias = {0.0f, 0.0f, 0.0f};
            initializeOrientationFromAccel(fusedAccel);
            initializeCovariance();
            filterInitialized = true;
            pendingReinitializeAfterCalibration = false;
        }

        const uint32_t mekfStartUs = tap::arch::clock::getTimeMicroseconds();
        predictWithGyro(fusedGyro, samplePeriodS);

        (void)runAccelUpdate(fusedAccel, fusedAccelVarianceDiag);

        updateEulerFromQuaternion();
        imuData.gyroRadPerSec = tap::algorithms::transforms::Vector(
            fusedGyro.x() - gyroBias[0],
            fusedGyro.y() - gyroBias[1],
            fusedGyro.z() - gyroBias[2]);
        imuData.accG = tap::algorithms::transforms::Vector(
            fusedAccel.x() - accelBias[0],
            fusedAccel.y() - accelBias[1],
            fusedAccel.z() - accelBias[2]);
        imuData.temperature = (tempCount > 0) ? (tempSum / tempCount) : 0.0f;
        if (telemetry != nullptr)
        {
            telemetry->logSignal(
                "perf/fused_imu/mekf_us",
                tap::arch::clock::getTimeMicroseconds() - mekfStartUs);
        }
        logCycleTime("perf/fused_imu/periodic_total_us");
    }

    inline const char* getName() const override { return "FusedIMUMEKFKF"; }
    inline float getAccelerationSensitivity() const override
    {
        return tap::communication::sensors::imu::GRAVITY_MPS2;
    }

    inline float getYaw() const override { return Angle(yawRad).getWrappedValue(); }
    inline float getPitch() const override { return Angle(pitchRad).getWrappedValue(); }
    inline float getRoll() const override { return Angle(rollRad).getWrappedValue(); }

private:
    static constexpr size_t signalStateSize = 6;
    static constexpr size_t errorStateSize = 9;
    static constexpr size_t quatSize = 4;
    using SignalFilterWrapper = aruwsrc::communication::sensors::imu::FusedImuEigenEkf<N>;
    using SignalStateMatrix = typename SignalFilterWrapper::StateMatrix;
    using SignalInputMatrix = typename SignalFilterWrapper::InputMatrix;
    using SignalInputVector = typename SignalFilterWrapper::InputVector;

    Config config;
    std::array<tap::communication::sensors::imu::AbstractIMU*, N> imus;
    std::array<tap::algorithms::transforms::Transform, N> imuTransforms;
    std::array<tap::algorithms::transforms::Transform, N> imuToFusionTransforms;
    std::array<typename Config::ImuNoiseDensity, N> perImuNoise;
    std::array<std::array<float, 3>, N> accelVar{};
    std::array<std::array<float, 3>, N> gyroVar{};
    std::array<std::array<float, 3>, N> baseAccelVariance{};
    std::array<std::array<float, 3>, N> baseGyroVariance{};
    std::array<float, 3> fusedAccelVarianceDiag = {1.0e-3f, 1.0e-3f, 1.0e-3f};
    std::array<float, 3> fusedGyroVarianceDiag = {1.0e-4f, 1.0e-4f, 1.0e-4f};
    float accelInnovationGateSq = 400.0f;
    float gyroInnovationGateSq = 16.0f;
    SignalFilterWrapper signalFilter;
    aruwsrc::communication::rtt::RttTelemetry* telemetry = nullptr;

    float samplePeriodS = 0.001f;
    uint32_t prevFilterUpdateTimeUs = 0U;
    bool signalFilterInitialized = false;
    bool filterInitialized = false;
    bool pendingReinitializeAfterCalibration = false;

    // Nominal state
    std::array<float, quatSize> q = {1.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 3> gyroBias = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> accelBias = {0.0f, 0.0f, 0.0f};
    // Error covariance P (9x9 row-major)
    std::array<float, errorStateSize * errorStateSize> P{};

    float rollRad = 0.0f;
    float pitchRad = 0.0f;
    float yawRad = 0.0f;

    // dont hate me chinmay
    static inline float wrapAngle(float x)
    {
        while (x >= M_PI) x -= M_TWOPI;
        while (x < -M_PI) x += M_TWOPI;
        return x;
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

    template <typename MatrixT>
    static inline void zeroMatrix(MatrixT& m)
    {
        SignalFilterWrapper::zeroMatrix(m);
    }

    template <typename MatrixT>
    static inline void setMatrixElem(MatrixT& m, size_t row, size_t col, float value)
    {
        SignalFilterWrapper::setMatrixElement(m, row, col, value);
    }

    template <typename VectorT>
    static inline void setVectorElem(VectorT& v, size_t row, float value)
    {
        SignalFilterWrapper::setVectorElement(v, row, value);
    }

    template <typename MatrixT>
    static inline MatrixT makeSignalStateDiagMatrix(const std::array<float, signalStateSize>& diag)
    {
        MatrixT mat{};
        zeroMatrix(mat);
        for (size_t i = 0; i < signalStateSize; i++)
        {
            setMatrixElem(mat, i, i, diag[i]);
        }
        return mat;
    }

    inline SignalStateMatrix makeSignalQ()
    {
        std::array<float, signalStateSize> qDiag = {
            config.accelProcessVarianceRateDiag[0],
            config.accelProcessVarianceRateDiag[1],
            config.accelProcessVarianceRateDiag[2],
            config.gyroProcessVarianceRateDiag[0],
            config.gyroProcessVarianceRateDiag[1],
            config.gyroProcessVarianceRateDiag[2]};
        return makeSignalStateDiagMatrix<SignalStateMatrix>(qDiag);
    }

    inline std::array<SignalInputMatrix, N> makeSignalR()
    {
        std::array<SignalInputMatrix, N> rBlocks{};
        for (size_t imuIndex = 0; imuIndex < N; imuIndex++)
        {
            auto& rBlock = rBlocks[imuIndex];
            zeroMatrix(rBlock);
            std::array<float, 3> accVar{};
            std::array<float, 3> gyrVar{};
            measurementVarianceDiagForImu(imuIndex, accVar, gyrVar);
            setMatrixElem(rBlock, 0, 0, accVar[0]);
            setMatrixElem(rBlock, 1, 1, accVar[1]);
            setMatrixElem(rBlock, 2, 2, accVar[2]);
            setMatrixElem(rBlock, 3, 3, gyrVar[0]);
            setMatrixElem(rBlock, 4, 4, gyrVar[1]);
            setMatrixElem(rBlock, 5, 5, gyrVar[2]);
        }
        return rBlocks;
    }

    inline SignalStateMatrix makeSignalP0()
    {
        return makeSignalStateDiagMatrix<SignalStateMatrix>(config.initialSignalStateVarianceDiag);
    }

    inline float effectiveNoiseBandwidthHz() const
    {
        const float dt = (samplePeriodS > 1.0e-9f) ? samplePeriodS : 1.0e-3f;
        const float fs = 1.0f / dt;
        float bw = 0.5f * fs;
        if (bw < config.minEffectiveNoiseBandwidthHz) bw = config.minEffectiveNoiseBandwidthHz;
        if (bw > config.maxEffectiveNoiseBandwidthHz) bw = config.maxEffectiveNoiseBandwidthHz;
        return bw;
    }

    inline void updatePerImuNoiseVariance()
    {
        const float bw = effectiveNoiseBandwidthHz();
        for (size_t i = 0; i < N; i++)
        {
            for (size_t a = 0; a < 3; a++)
            {
                const float ndAcc = (perImuNoise[i].accelNoiseDensityUgSqrtHz[a] * 1.0e-6f) *
                                    tap::communication::sensors::imu::GRAVITY_MPS2;
                accelVar[i][a] = ndAcc * ndAcc * bw;

                const float ndGyro =
                    modm::toRadian(perImuNoise[i].gyroNoiseDensityMdpsSqrtHz[a] * 1.0e-3f);
                gyroVar[i][a] = ndGyro * ndGyro * bw;
                baseAccelVariance[i][a] = accelVar[i][a];
                baseGyroVariance[i][a] = gyroVar[i][a];
            }
        }
    }

    inline void updateFusedMeasurementVariance(const std::array<bool, N>& validFlags)
    {
        constexpr float minVar = 1.0e-12f;
        for (size_t axis = 0; axis < 3; axis++)
        {
            float accelPrecisionSum = 0.0f;
            float gyroPrecisionSum = 0.0f;
            bool anyAccelValid = false;
            bool anyGyroValid = false;
            for (size_t i = 0; i < N; i++)
            {
                if (!validFlags[i])
                {
                    continue;
                }
                const float accelAxisVar = std::clamp(accelVar[i][axis], minVar, 1.0e8f);
                const float gyroAxisVar = std::clamp(gyroVar[i][axis], minVar, 1.0e8f);
                accelPrecisionSum += 1.0f / accelAxisVar;
                gyroPrecisionSum += 1.0f / gyroAxisVar;
                anyAccelValid = true;
                anyGyroValid = true;
            }

            if (anyAccelValid && accelPrecisionSum > 0.0f)
            {
                fusedAccelVarianceDiag[axis] = 1.0f / accelPrecisionSum;
            }
            else
            {
                float accelMean = 0.0f;
                for (size_t i = 0; i < N; i++)
                {
                    accelMean += std::clamp(accelVar[i][axis], minVar, 1.0e8f);
                }
                fusedAccelVarianceDiag[axis] = accelMean / static_cast<float>(N);
            }

            if (anyGyroValid && gyroPrecisionSum > 0.0f)
            {
                fusedGyroVarianceDiag[axis] = 1.0f / gyroPrecisionSum;
            }
            else
            {
                float gyroMean = 0.0f;
                for (size_t i = 0; i < N; i++)
                {
                    gyroMean += std::clamp(gyroVar[i][axis], minVar, 1.0e8f);
                }
                fusedGyroVarianceDiag[axis] = gyroMean / static_cast<float>(N);
            }
        }
    }

    inline void measurementVarianceDiagForImu(
        size_t imuIndex,
        std::array<float, 3>& accelVarDiagOut,
        std::array<float, 3>& gyroVarDiagOut) const
    {
        const float bw = effectiveNoiseBandwidthHz();
        for (size_t a = 0; a < 3; a++)
        {
            const float ndAcc = (perImuNoise[imuIndex].accelNoiseDensityUgSqrtHz[a] * 1.0e-6f) *
                                tap::communication::sensors::imu::GRAVITY_MPS2;
            accelVarDiagOut[a] = ndAcc * ndAcc * bw;

            const float ndGyro =
                modm::toRadian(perImuNoise[imuIndex].gyroNoiseDensityMdpsSqrtHz[a] * 1.0e-3f);
            gyroVarDiagOut[a] = ndGyro * ndGyro * bw;
        }
    }

    inline void updateSignalMeasurementCovariance(
        const std::array<tap::communication::sensors::imu::ImuInterface::ImuState, N>& states,
        const std::array<tap::algorithms::transforms::Vector, N>& accel,
        const std::array<tap::algorithms::transforms::Vector, N>& gyro,
        const std::array<bool, N>& validFlags)
    {
        auto& rBlocks = signalFilter.getMeasurementCovarianceBlocks();
        const auto& x = signalFilter.getStateVectorAsMatrix();
        const float predAx = x[0];
        const float predAy = x[1];
        const float predAz = x[2];
        const float predGx = x[3];
        const float predGy = x[4];
        const float predGz = x[5];

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
            else if (signalFilterInitialized)
            {
                const float dax = accel[imuIndex].x() - predAx;
                const float day = accel[imuIndex].y() - predAy;
                const float daz = accel[imuIndex].z() - predAz;
                const float dgx = gyro[imuIndex].x() - predGx;
                const float dgy = gyro[imuIndex].y() - predGy;
                const float dgz = gyro[imuIndex].z() - predGz;
                const float accelResidualSq = dax * dax + day * day + daz * daz;
                const float gyroResidualSq = dgx * dgx + dgy * dgy + dgz * dgz;

                if (accelResidualSq > accelInnovationGateSq)
                {
                    const float ratioSq = accelResidualSq / accelInnovationGateSq;
                    const float adaptiveScale = ratioSq * config.outlierVarianceMultiplier;
                    accelMultiplier =
                        std::fmin(adaptiveScale, config.maxInnovationVarianceMultiplier);
                }
                if (gyroResidualSq > gyroInnovationGateSq)
                {
                    const float ratioSq = gyroResidualSq / gyroInnovationGateSq;
                    const float adaptiveScale = ratioSq * config.outlierVarianceMultiplier;
                    gyroMultiplier =
                        std::fmin(adaptiveScale, config.maxInnovationVarianceMultiplier);
                }
            }

            auto& rBlock = rBlocks[imuIndex];
            float r00 = baseAccelVariance[imuIndex][0] * accelMultiplier;
            float r11 = baseAccelVariance[imuIndex][1] * accelMultiplier;
            float r22 = baseAccelVariance[imuIndex][2] * accelMultiplier;
            float r33 = baseGyroVariance[imuIndex][0] * gyroMultiplier;
            float r44 = baseGyroVariance[imuIndex][1] * gyroMultiplier;
            float r55 = baseGyroVariance[imuIndex][2] * gyroMultiplier;
            r00 = std::clamp(r00, config.minMeasurementVariance, config.maxMeasurementVariance);
            r11 = std::clamp(r11, config.minMeasurementVariance, config.maxMeasurementVariance);
            r22 = std::clamp(r22, config.minMeasurementVariance, config.maxMeasurementVariance);
            r33 = std::clamp(r33, config.minMeasurementVariance, config.maxMeasurementVariance);
            r44 = std::clamp(r44, config.minMeasurementVariance, config.maxMeasurementVariance);
            r55 = std::clamp(r55, config.minMeasurementVariance, config.maxMeasurementVariance);
            setMatrixElem(rBlock, 0, 0, r00);
            setMatrixElem(rBlock, 1, 1, r11);
            setMatrixElem(rBlock, 2, 2, r22);
            setMatrixElem(rBlock, 3, 3, r33);
            setMatrixElem(rBlock, 4, 4, r44);
            setMatrixElem(rBlock, 5, 5, r55);
        }
    }

    inline void updateSignalProcessCovariance(float dt)
    {
        const float clampedDt = (dt > 1.0e-6f) ? dt : 1.0e-3f;
        auto& qSignal = signalFilter.getProcessCovariance();
        qSignal[0 * signalStateSize + 0] = config.accelProcessVarianceRateDiag[0] * clampedDt;
        qSignal[1 * signalStateSize + 1] = config.accelProcessVarianceRateDiag[1] * clampedDt;
        qSignal[2 * signalStateSize + 2] = config.accelProcessVarianceRateDiag[2] * clampedDt;
        qSignal[3 * signalStateSize + 3] = config.gyroProcessVarianceRateDiag[0] * clampedDt;
        qSignal[4 * signalStateSize + 4] = config.gyroProcessVarianceRateDiag[1] * clampedDt;
        qSignal[5 * signalStateSize + 5] = config.gyroProcessVarianceRateDiag[2] * clampedDt;
    }

    inline static void quatNormalize(std::array<float, 4>& qInOut)
    {
        const float n = std::sqrt(
            qInOut[0] * qInOut[0] + qInOut[1] * qInOut[1] + qInOut[2] * qInOut[2] +
            qInOut[3] * qInOut[3]);
        if (n <= 1.0e-9f)
        {
            qInOut = {1.0f, 0.0f, 0.0f, 0.0f};
            return;
        }
        const float inv = 1.0f / n;
        qInOut[0] *= inv;
        qInOut[1] *= inv;
        qInOut[2] *= inv;
        qInOut[3] *= inv;
    }

    inline static std::array<float, 4> quatMul(
        const std::array<float, 4>& a,
        const std::array<float, 4>& b)
    {
        return {
            a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
    }

    inline static std::array<float, 4> quatFromEuler(float roll, float pitch, float yaw)
    {
        const float cr = std::cos(0.5f * roll);
        const float sr = std::sin(0.5f * roll);
        const float cp = std::cos(0.5f * pitch);
        const float sp = std::sin(0.5f * pitch);
        const float cy = std::cos(0.5f * yaw);
        const float sy = std::sin(0.5f * yaw);
        return {
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy};
    }

    inline void updateEulerFromQuaternion()
    {
        const float w = q[0];
        const float x = q[1];
        const float y = q[2];
        const float z = q[3];

        const float sinr_cosp = 2.0f * (w * x + y * z);
        const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        rollRad = std::atan2(sinr_cosp, cosr_cosp);

        const float sinp = 2.0f * (w * y - z * x);
        if (std::fabs(sinp) >= 1.0f)
        {
            pitchRad = std::copysign(0.5f * static_cast<float>(M_PI), sinp);
        }
        else
        {
            pitchRad = std::asin(sinp);
        }

        const float siny_cosp = 2.0f * (w * z + x * y);
        const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        yawRad = std::atan2(siny_cosp, cosy_cosp);
    }

    inline std::array<float, 3> gravityBodyFromQuat() const
    {
        const float w = q[0];
        const float x = q[1];
        const float y = q[2];
        const float z = q[3];
        const float g = tap::communication::sensors::imu::GRAVITY_MPS2;
        // Match Mahony/Taproot quaternion convention:
        // gx = 2*(x*z - w*y), gy = 2*(w*x + y*z), gz = 1 - 2*(x^2 + y^2)
        return {
            2.0f * (x * z - w * y) * g,
            2.0f * (w * x + y * z) * g,
            (1.0f - 2.0f * (x * x + y * y)) * g};
    }

    inline void resetFilterState()
    {
        q = {1.0f, 0.0f, 0.0f, 0.0f};
        gyroBias = {0.0f, 0.0f, 0.0f};
        accelBias = {0.0f, 0.0f, 0.0f};
        for (size_t i = 0; i < P.size(); i++)
        {
            P[i] = 0.0f;
        }
        rollRad = 0.0f;
        pitchRad = 0.0f;
        yawRad = 0.0f;
    }

    inline void initializeCovariance()
    {
        for (size_t i = 0; i < P.size(); i++)
        {
            P[i] = 0.0f;
        }

        const float angleVar = config.initialAngleStdRad * config.initialAngleStdRad;
        const float gyroBiasVar =
            config.initialGyroBiasStdRadPerSec * config.initialGyroBiasStdRadPerSec;
        const float accelBiasVar = config.initialAccelBiasStdMps2 * config.initialAccelBiasStdMps2;

        P[0 * errorStateSize + 0] = angleVar;
        P[1 * errorStateSize + 1] = angleVar;
        P[2 * errorStateSize + 2] = angleVar;
        P[3 * errorStateSize + 3] = gyroBiasVar;
        P[4 * errorStateSize + 4] = gyroBiasVar;
        P[5 * errorStateSize + 5] = gyroBiasVar;
        P[6 * errorStateSize + 6] = accelBiasVar;
        P[7 * errorStateSize + 7] = accelBiasVar;
        P[8 * errorStateSize + 8] = accelBiasVar;
    }

    inline void enforceCovarianceNumerics()
    {
        for (size_t i = 0; i < P.size(); i++)
        {
            if (!std::isfinite(P[i]))
            {
                P[i] = 0.0f;
            }
        }

        for (size_t r = 0; r < errorStateSize; r++)
        {
            for (size_t c = r + 1; c < errorStateSize; c++)
            {
                const float sym = 0.5f * (P[r * errorStateSize + c] + P[c * errorStateSize + r]);
                P[r * errorStateSize + c] = sym;
                P[c * errorStateSize + r] = sym;
            }
        }

        constexpr float minDiag = 1.0e-10f;
        constexpr float maxDiag = 1.0e6f;
        for (size_t d = 0; d < errorStateSize; d++)
        {
            const size_t idx = d * errorStateSize + d;
            P[idx] = std::clamp(P[idx], minDiag, maxDiag);
        }
    }

    inline void initializeOrientationFromAccel(const tap::algorithms::transforms::Vector& acc)
    {
        const float ax = acc.x();
        const float ay = acc.y();
        const float az = acc.z();
        const float roll = std::atan2(ay, az);
        const float pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
        q = quatFromEuler(roll, pitch, 0.0f);
        quatNormalize(q);
        updateEulerFromQuaternion();
    }

    inline void predictWithGyro(const tap::algorithms::transforms::Vector& gyroMeas, float dt)
    {
        const float wx = gyroMeas.x() - gyroBias[0];
        const float wy = gyroMeas.y() - gyroBias[1];
        const float wz = gyroMeas.z() - gyroBias[2];

        const std::array<float, 4> omegaQ = {0.0f, wx, wy, wz};
        std::array<float, 4> dq = quatMul(q, omegaQ);
        for (size_t i = 0; i < 4; i++)
        {
            q[i] += 0.5f * dq[i] * dt;
        }
        quatNormalize(q);

        const float dtwx = wx * dt;
        const float dtwy = wy * dt;
        const float dtwz = wz * dt;
        // Phi = I + A, where A has non-zeros only in rows 0..2.
        const float a01 = dtwz;
        const float a02 = -dtwy;
        const float a10 = -dtwz;
        const float a12 = dtwx;
        const float a20 = dtwy;
        const float a21 = -dtwx;
        const float negDt = -dt;
        std::array<float, errorStateSize * errorStateSize> tmp;
        std::array<float, errorStateSize * errorStateSize> pNew;
        // tmp = Phi * P (sparse row update on first 3 rows)
        for (size_t c = 0; c < errorStateSize; c++)
        {
            const float p0c = P[0 * errorStateSize + c];
            const float p1c = P[1 * errorStateSize + c];
            const float p2c = P[2 * errorStateSize + c];
            tmp[0 * errorStateSize + c] =
                p0c + a01 * p1c + a02 * p2c + negDt * P[3 * errorStateSize + c];
            tmp[1 * errorStateSize + c] =
                a10 * p0c + p1c + a12 * p2c + negDt * P[4 * errorStateSize + c];
            tmp[2 * errorStateSize + c] =
                a20 * p0c + a21 * p1c + p2c + negDt * P[5 * errorStateSize + c];
        }
        for (size_t r = 3; r < errorStateSize; r++)
        {
            for (size_t c = 0; c < errorStateSize; c++)
            {
                tmp[r * errorStateSize + c] = P[r * errorStateSize + c];
            }
        }

        // pNew = tmp * Phi^T (sparse column update on first 3 cols)
        for (size_t r = 0; r < errorStateSize; r++)
        {
            const float tr0 = tmp[r * errorStateSize + 0];
            const float tr1 = tmp[r * errorStateSize + 1];
            const float tr2 = tmp[r * errorStateSize + 2];
            pNew[r * errorStateSize + 0] =
                tr0 + a01 * tr1 + a02 * tr2 + negDt * tmp[r * errorStateSize + 3];
            pNew[r * errorStateSize + 1] =
                a10 * tr0 + tr1 + a12 * tr2 + negDt * tmp[r * errorStateSize + 4];
            pNew[r * errorStateSize + 2] =
                a20 * tr0 + a21 * tr1 + tr2 + negDt * tmp[r * errorStateSize + 5];
            for (size_t c = 3; c < errorStateSize; c++)
            {
                pNew[r * errorStateSize + c] = tmp[r * errorStateSize + c];
            }
        }

        // Qd diagonal
        const float gyroVarAvg =
            (fusedGyroVarianceDiag[0] + fusedGyroVarianceDiag[1] + fusedGyroVarianceDiag[2]) *
            (1.0f / 3.0f);
        const float qTheta = std::clamp(gyroVarAvg * dt, 1.0e-12f, 1.0f);
        const float qBg = std::clamp(
            config.gyroBiasRandomWalkStdRadPerSec * config.gyroBiasRandomWalkStdRadPerSec * dt,
            1.0e-14f,
            1.0f);
        const float qBa = std::clamp(
            config.accelBiasRandomWalkStdMps2 * config.accelBiasRandomWalkStdMps2 * dt,
            1.0e-14f,
            1.0f);
        pNew[0 * errorStateSize + 0] += qTheta;
        pNew[1 * errorStateSize + 1] += qTheta;
        pNew[2 * errorStateSize + 2] += qTheta;
        pNew[3 * errorStateSize + 3] += qBg;
        pNew[4 * errorStateSize + 4] += qBg;
        pNew[5 * errorStateSize + 5] += qBg;
        pNew[6 * errorStateSize + 6] += qBa;
        pNew[7 * errorStateSize + 7] += qBa;
        pNew[8 * errorStateSize + 8] += qBa;

        P = pNew;
        enforceCovarianceNumerics();
    }

    inline bool invert3x3(const float a[9], float invOut[9]) const
    {
        const float det = a[0] * (a[4] * a[8] - a[5] * a[7]) - a[1] * (a[3] * a[8] - a[5] * a[6]) +
                          a[2] * (a[3] * a[7] - a[4] * a[6]);
        if (std::fabs(det) < 1.0e-12f)
        {
            return false;
        }
        const float invDet = 1.0f / det;
        invOut[0] = (a[4] * a[8] - a[5] * a[7]) * invDet;
        invOut[1] = (a[2] * a[7] - a[1] * a[8]) * invDet;
        invOut[2] = (a[1] * a[5] - a[2] * a[4]) * invDet;
        invOut[3] = (a[5] * a[6] - a[3] * a[8]) * invDet;
        invOut[4] = (a[0] * a[8] - a[2] * a[6]) * invDet;
        invOut[5] = (a[2] * a[3] - a[0] * a[5]) * invDet;
        invOut[6] = (a[3] * a[7] - a[4] * a[6]) * invDet;
        invOut[7] = (a[1] * a[6] - a[0] * a[7]) * invDet;
        invOut[8] = (a[0] * a[4] - a[1] * a[3]) * invDet;
        return true;
    }

    inline bool runAccelUpdate(
        const tap::algorithms::transforms::Vector& accelMeas,
        const std::array<float, 3>& accelVarDiag)
    {
        const float ax = accelMeas.x();
        const float ay = accelMeas.y();
        const float az = accelMeas.z();

        const float norm = std::sqrt(ax * ax + ay * ay + az * az);
        if (norm < config.minAccelNormMps2 || norm > config.maxAccelNormMps2)
        {
            return false;
        }

        const float g = tap::communication::sensors::imu::GRAVITY_MPS2;
        if (std::fabs(norm - g) > config.accelGateMps2)
        {
            return false;
        }

        const auto gBody = gravityBodyFromQuat();
        const float gxBody = gBody[0];
        const float gyBody = gBody[1];
        const float gzBody = gBody[2];
        const float h[3] = {gxBody + accelBias[0], gyBody + accelBias[1], gzBody + accelBias[2]};
        const float r[3] = {ax - h[0], ay - h[1], az - h[2]};
        const float normError = std::fabs(norm - g);
        const float innovationNorm = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
        const float dynamicVarianceScale = std::clamp(
            1.0f + config.accelDynamicVarianceGain * (normError * normError + innovationNorm),
            1.0f,
            config.accelDynamicVarianceMaxScale);

        // H = [ skew(gBody) 0 I ] has non-zeros only in cols {0,1,2,6,7,8}.
        // Compute PHt = P * H^T using this sparsity.
        float PHt[errorStateSize * 3];
        for (size_t i = 0; i < errorStateSize; i++)
        {
            const float pi0 = P[i * errorStateSize + 0];
            const float pi1 = P[i * errorStateSize + 1];
            const float pi2 = P[i * errorStateSize + 2];
            PHt[i * 3 + 0] = -pi1 * gzBody + pi2 * gyBody + P[i * errorStateSize + 6];
            PHt[i * 3 + 1] = pi0 * gzBody - pi2 * gxBody + P[i * errorStateSize + 7];
            PHt[i * 3 + 2] = -pi0 * gyBody + pi1 * gxBody + P[i * errorStateSize + 8];
        }

        // S = H*PHt + R (3x3)
        float S[9];
        S[0] = -gzBody * PHt[1 * 3 + 0] + gyBody * PHt[2 * 3 + 0] + PHt[6 * 3 + 0];
        S[1] = -gzBody * PHt[1 * 3 + 1] + gyBody * PHt[2 * 3 + 1] + PHt[6 * 3 + 1];
        S[2] = -gzBody * PHt[1 * 3 + 2] + gyBody * PHt[2 * 3 + 2] + PHt[6 * 3 + 2];
        S[3] = gzBody * PHt[0 * 3 + 0] - gxBody * PHt[2 * 3 + 0] + PHt[7 * 3 + 0];
        S[4] = gzBody * PHt[0 * 3 + 1] - gxBody * PHt[2 * 3 + 1] + PHt[7 * 3 + 1];
        S[5] = gzBody * PHt[0 * 3 + 2] - gxBody * PHt[2 * 3 + 2] + PHt[7 * 3 + 2];
        S[6] = -gyBody * PHt[0 * 3 + 0] + gxBody * PHt[1 * 3 + 0] + PHt[8 * 3 + 0];
        S[7] = -gyBody * PHt[0 * 3 + 1] + gxBody * PHt[1 * 3 + 1] + PHt[8 * 3 + 1];
        S[8] = -gyBody * PHt[0 * 3 + 2] + gxBody * PHt[1 * 3 + 2] + PHt[8 * 3 + 2];
        const float accelVarianceScale =
            config.accelMeasurementVarianceScale * dynamicVarianceScale;
        const float r0 = std::clamp(accelVarDiag[0] * accelVarianceScale, 1.0e-8f, 1.0e5f);
        const float r1 = std::clamp(accelVarDiag[1] * accelVarianceScale, 1.0e-8f, 1.0e5f);
        const float r2 = std::clamp(accelVarDiag[2] * accelVarianceScale, 1.0e-8f, 1.0e5f);
        S[0] += r0;
        S[4] += r1;
        S[8] += r2;

        float SInv[9];
        if (!invert3x3(S, SInv))
        {
            return false;
        }

        const float sr0 = SInv[0] * r[0] + SInv[1] * r[1] + SInv[2] * r[2];
        const float sr1 = SInv[3] * r[0] + SInv[4] * r[1] + SInv[5] * r[2];
        const float sr2 = SInv[6] * r[0] + SInv[7] * r[1] + SInv[8] * r[2];
        const float nis = r[0] * sr0 + r[1] * sr1 + r[2] * sr2;
        if (nis > config.accelNisGate)
        {
            return false;
        }

        // K = PHt * SInv (9x3)
        float K[errorStateSize * 3];
        for (size_t i = 0; i < errorStateSize; i++)
        {
            for (size_t j = 0; j < 3; j++)
            {
                float s = 0.0f;
                for (size_t k = 0; k < 3; k++)
                {
                    s += PHt[i * 3 + k] * SInv[k * 3 + j];
                }
                K[i * 3 + j] = s;
            }
        }

        // Apply correction gains directly to Kalman gain rows so state and covariance
        // updates remain internally consistent.
        for (size_t k = 0; k < 3; k++)
        {
            K[0 * 3 + k] *= config.attitudeCorrectionGain;
            K[1 * 3 + k] *= config.attitudeCorrectionGain;
            K[2 * 3 + k] *= config.attitudeCorrectionGain;
            K[3 * 3 + k] *= config.gyroBiasCorrectionGain;
            K[4 * 3 + k] *= config.gyroBiasCorrectionGain;
            K[5 * 3 + k] *= config.gyroBiasCorrectionGain;
            K[6 * 3 + k] *= config.accelBiasCorrectionGain;
            K[7 * 3 + k] *= config.accelBiasCorrectionGain;
            K[8 * 3 + k] *= config.accelBiasCorrectionGain;
        }

        // Remove unobservable yaw-like attitude correction from accelerometer update by
        // projecting attitude-gain rows onto the plane normal to gravity.
        if (config.suppressYawCorrectionFromAccel)
        {
            const float gNormSq = gBody[0] * gBody[0] + gBody[1] * gBody[1] + gBody[2] * gBody[2];
            if (gNormSq > 1.0e-8f)
            {
                const float invGNorm = 1.0f / std::sqrt(gNormSq);
                const float gux = gBody[0] * invGNorm;
                const float guy = gBody[1] * invGNorm;
                const float guz = gBody[2] * invGNorm;
                for (size_t k = 0; k < 3; k++)
                {
                    const float kx = K[0 * 3 + k];
                    const float ky = K[1 * 3 + k];
                    const float kz = K[2 * 3 + k];
                    const float yawLikeComp = kx * gux + ky * guy + kz * guz;
                    K[0 * 3 + k] = kx - yawLikeComp * gux;
                    K[1 * 3 + k] = ky - yawLikeComp * guy;
                    K[2 * 3 + k] = kz - yawLikeComp * guz;
                }
            }
        }
        // State correction
        const float dthx = K[0 * 3 + 0] * r[0] + K[0 * 3 + 1] * r[1] + K[0 * 3 + 2] * r[2];
        const float dthy = K[1 * 3 + 0] * r[0] + K[1 * 3 + 1] * r[1] + K[1 * 3 + 2] * r[2];
        const float dthz = K[2 * 3 + 0] * r[0] + K[2 * 3 + 1] * r[1] + K[2 * 3 + 2] * r[2];
        const float dbgx = K[3 * 3 + 0] * r[0] + K[3 * 3 + 1] * r[1] + K[3 * 3 + 2] * r[2];
        const float dbgy = K[4 * 3 + 0] * r[0] + K[4 * 3 + 1] * r[1] + K[4 * 3 + 2] * r[2];
        const float dbgz = K[5 * 3 + 0] * r[0] + K[5 * 3 + 1] * r[1] + K[5 * 3 + 2] * r[2];
        const float dbax = K[6 * 3 + 0] * r[0] + K[6 * 3 + 1] * r[1] + K[6 * 3 + 2] * r[2];
        const float dbay = K[7 * 3 + 0] * r[0] + K[7 * 3 + 1] * r[1] + K[7 * 3 + 2] * r[2];
        const float dbaz = K[8 * 3 + 0] * r[0] + K[8 * 3 + 1] * r[1] + K[8 * 3 + 2] * r[2];
        const std::array<float, 4> dq = {1.0f, 0.5f * dthx, 0.5f * dthy, 0.5f * dthz};
        q = quatMul(q, dq);
        quatNormalize(q);
        gyroBias[0] = std::clamp(
            gyroBias[0] + dbgx,
            -config.maxGyroBiasAbsRadPerSec,
            config.maxGyroBiasAbsRadPerSec);
        gyroBias[1] = std::clamp(
            gyroBias[1] + dbgy,
            -config.maxGyroBiasAbsRadPerSec,
            config.maxGyroBiasAbsRadPerSec);
        gyroBias[2] = std::clamp(
            gyroBias[2] + dbgz,
            -config.maxGyroBiasAbsRadPerSec,
            config.maxGyroBiasAbsRadPerSec);
        accelBias[0] = std::clamp(
            accelBias[0] + dbax,
            -config.maxAccelBiasAbsMps2,
            config.maxAccelBiasAbsMps2);
        accelBias[1] = std::clamp(
            accelBias[1] + dbay,
            -config.maxAccelBiasAbsMps2,
            config.maxAccelBiasAbsMps2);
        accelBias[2] = std::clamp(
            accelBias[2] + dbaz,
            -config.maxAccelBiasAbsMps2,
            config.maxAccelBiasAbsMps2);

        // Covariance update in Joseph form:
        // P = (I - K H) P (I - K H)^T + K R K^T
        // This remains valid even after gain shaping
        // more expensive tho
        float H[3 * errorStateSize] = {};
        H[0 * errorStateSize + 1] = -gzBody;
        H[0 * errorStateSize + 2] = gyBody;
        H[0 * errorStateSize + 6] = 1.0f;
        H[1 * errorStateSize + 0] = gzBody;
        H[1 * errorStateSize + 2] = -gxBody;
        H[1 * errorStateSize + 7] = 1.0f;
        H[2 * errorStateSize + 0] = -gyBody;
        H[2 * errorStateSize + 1] = gxBody;
        H[2 * errorStateSize + 8] = 1.0f;

        std::array<float, errorStateSize * errorStateSize> iMinusKH{};
        for (size_t i = 0; i < errorStateSize; i++)
        {
            iMinusKH[i * errorStateSize + i] = 1.0f;
        }
        for (size_t i = 0; i < errorStateSize; i++)
        {
            for (size_t j = 0; j < errorStateSize; j++)
            {
                float s = 0.0f;
                for (size_t k = 0; k < 3; k++)
                {
                    s += K[i * 3 + k] * H[k * errorStateSize + j];
                }
                iMinusKH[i * errorStateSize + j] -= s;
            }
        }

        std::array<float, errorStateSize * errorStateSize> tmp{};
        std::array<float, errorStateSize * errorStateSize> pNew{};
        for (size_t r = 0; r < errorStateSize; r++)
        {
            for (size_t c = 0; c < errorStateSize; c++)
            {
                float s = 0.0f;
                for (size_t k = 0; k < errorStateSize; k++)
                {
                    s += iMinusKH[r * errorStateSize + k] * P[k * errorStateSize + c];
                }
                tmp[r * errorStateSize + c] = s;
            }
        }
        for (size_t r = 0; r < errorStateSize; r++)
        {
            for (size_t c = 0; c < errorStateSize; c++)
            {
                float s = 0.0f;
                for (size_t k = 0; k < errorStateSize; k++)
                {
                    s += tmp[r * errorStateSize + k] * iMinusKH[c * errorStateSize + k];
                }
                pNew[r * errorStateSize + c] = s;
            }
        }

        const float rDiag[3] = {r0, r1, r2};
        for (size_t r = 0; r < errorStateSize; r++)
        {
            for (size_t c = 0; c < errorStateSize; c++)
            {
                float s = 0.0f;
                for (size_t k = 0; k < 3; k++)
                {
                    s += K[r * 3 + k] * rDiag[k] * K[c * 3 + k];
                }
                pNew[r * errorStateSize + c] += s;
            }
        }

        P = pNew;
        enforceCovarianceNumerics();
        return true;
    }

    inline tap::algorithms::transforms::Vector transformAcceleration(
        const tap::algorithms::transforms::Transform& fusionToImu,
        const tap::algorithms::transforms::Transform& imuToFusion,
        const tap::algorithms::transforms::Vector& imuAcc) const
    {
        const auto imuPosition = fusionToImu.getTranslation();
        const auto fusionAngVel = fusionToImu.getAngularVel();
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
        const auto fusedDynamics = imuToFusion.apply(imuDynamics);
        return fusedDynamics.getAcceleration();
    }

    inline tap::algorithms::transforms::Vector transformGyro(
        const tap::algorithms::transforms::Transform& fusionToImu,
        const tap::algorithms::transforms::Transform& imuToFusion,
        const tap::algorithms::transforms::Vector& imuGyro) const
    {
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

    static tap::communication::sensors::imu::ImuInterface::ImuState combineImuStates(
        const std::array<tap::communication::sensors::imu::ImuInterface::ImuState, N>& states)
    {
        bool anyConnected = false;
        for (auto state : states)
        {
            if (state == tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATING)
            {
                return tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATING;
            }
            if (state ==
                    tap::communication::sensors::imu::ImuInterface::ImuState::IMU_NOT_CALIBRATED ||
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
            if (state ==
                tap::communication::sensors::imu::ImuInterface::ImuState::IMU_NOT_CALIBRATED)
            {
                return tap::communication::sensors::imu::ImuInterface::ImuState::IMU_NOT_CALIBRATED;
            }
        }
        return tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATED;
    }

    inline void recomputeImuToFusionTransform(size_t index)
    {
        if (index >= N)
        {
            return;
        }
        imuToFusionTransforms[index] = mountingTransform.compose(imuTransforms[index].getInverse());
    }

    inline void recomputeImuToFusionTransforms()
    {
        for (size_t i = 0; i < N; i++)
        {
            recomputeImuToFusionTransform(i);
        }
    }
};
}  // namespace aruwsrc::communication::sensors::imu

#endif  // FUSED_IMU_MEKF_KF_HPP_
