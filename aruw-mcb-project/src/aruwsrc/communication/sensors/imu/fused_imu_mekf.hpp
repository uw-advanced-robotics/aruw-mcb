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

#ifndef FUSED_IMU_MEKF_HPP_
#define FUSED_IMU_MEKF_HPP_

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

namespace aruwsrc::communication::sensors::imu
{
template <size_t N>
class FusedImuMekf final : public tap::communication::sensors::imu::AbstractIMU
{
public:
    static_assert(N > 0, "FusedImuMekf requires at least one IMU");
    mutable volatile uint32_t debugState = 0U;

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

        float gyroBiasRandomWalkStdRadPerSec = 2.0e-4f;
        float accelBiasRandomWalkStdMps2 = 4.0e-3f;
        float initialAngleStdRad = 0.2f;
        float initialGyroBiasStdRadPerSec = 0.1f;
        float initialAccelBiasStdMps2 = 0.5f;

        // Conservative defaults: behave close to weighted averaging + AHRS.
        bool enableGyroBiasEstimation = false;
        bool enableAccelBiasEstimation = false;
        float accelMeasurementVarianceScale = 8.0f;
        float attitudeCorrectionGain = 0.10f;
        float gyroBiasCorrectionGain = 0.01f;
        float accelBiasCorrectionGain = 0.0f;
    };

    struct TimingProfile
    {
        uint32_t totalUs = 0;
        uint32_t collectUs = 0;
        uint32_t predictUs = 0;
        uint32_t updateUs = 0;
        uint32_t mahonyUs = 0;
        uint16_t updates = 0;
    };

    using TimingTelemetryCallback = void (*)(const char* label, uint32_t value);

    FusedImuMekf(
        const std::array<tap::communication::sensors::imu::AbstractIMU*, N>& imus,
        const std::array<tap::algorithms::transforms::Transform, N>& transforms,
        const std::array<ImuType, N>& imuTypes,
        const Config& config = Config())
        : AbstractIMU(tap::algorithms::transforms::Transform::identity()),
          config(config),
          imus(imus),
          imuTransforms(transforms),
          imuToFusionTransforms(transforms),
          perImuNoise(selectPerImuNoise(imuTypes, config))
    {
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
        filterInitialized = false;
        pendingReinitializeAfterCalibration = true;
        requestCalibration();
    }

    void requestCalibration() override
    {
        filterInitialized = false;
        pendingReinitializeAfterCalibration = true;
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
        debugState = 10U;
        const uint32_t cycleStartUs = tap::arch::clock::getTimeMicroseconds();
        uint32_t stageStartUs = cycleStartUs;
        timingProfile = {};

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
        debugState = 20U;
        imuState = combineImuStates(states);
        if (imuState == tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATING)
        {
            debugState = 21U;
            pendingReinitializeAfterCalibration = true;
            filterInitialized = false;
            imuData.accG = tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f);
            imuData.gyroRadPerSec = tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f);
            imuData.temperature = 0.0f;
            emitTiming(cycleStartUs, tap::arch::clock::getTimeMicroseconds());
            return;
        }

        auto accel = makeVectorArray(tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f));
        auto gyro = makeVectorArray(tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f));
        std::array<bool, N> validFlags{};
        bool anyValid = false;
        float tempSum = 0.0f;
        int tempCount = 0;

        for (size_t i = 0; i < N; i++)
        {
            debugState = 30U + static_cast<uint32_t>(i);
            const bool valid = isConnected(states[i]) && isValid(states[i]);
            debugState = 130U + static_cast<uint32_t>(i);
            validFlags[i] = valid;
            if (!valid)
            {
                debugState = 230U + static_cast<uint32_t>(i);
                continue;
            }

            debugState = 330U + static_cast<uint32_t>(i);
            const tap::algorithms::transforms::Vector imuAcc(
                imus[i]->getAx(),
                imus[i]->getAy(),
                imus[i]->getAz());
            debugState = 430U + static_cast<uint32_t>(i);
            const tap::algorithms::transforms::Vector imuGyro(
                imus[i]->getGx(),
                imus[i]->getGy(),
                imus[i]->getGz());
            debugState = 530U + static_cast<uint32_t>(i);

            accel[i] = transformAcceleration(imuTransforms[i], imuToFusionTransforms[i], imuAcc);
            debugState = 630U + static_cast<uint32_t>(i);
            gyro[i] = transformGyro(imuTransforms[i], imuToFusionTransforms[i], imuGyro);
            debugState = 730U + static_cast<uint32_t>(i);
            tempSum += imus[i]->getTemp();
            tempCount++;
            anyValid = true;
        }

        uint32_t nowUs = tap::arch::clock::getTimeMicroseconds();
        timingProfile.collectUs = nowUs - stageStartUs;
        stageStartUs = nowUs;

        if (!anyValid)
        {
            debugState = 39U;
            imuData.accG = tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f);
            imuData.gyroRadPerSec = tap::algorithms::transforms::Vector(0.0f, 0.0f, 0.0f);
            imuData.temperature = 0.0f;
            emitTiming(cycleStartUs, tap::arch::clock::getTimeMicroseconds());
            return;
        }

        if (pendingReinitializeAfterCalibration &&
            imuState != tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATING)
        {
            filterInitialized = false;
        }

        updatePerImuNoiseVariance();
        debugState = 40U;
        auto fusedGyro = weightedAverage(gyro, validFlags, gyroVar);
        debugState = 41U;
        auto fusedAccel = weightedAverage(accel, validFlags, accelVar);
        debugState = 42U;

        if (!filterInitialized)
        {
            initializeOrientationFromAccel(fusedAccel);
            gyroBias = {0.0f, 0.0f, 0.0f};
            accelBias = {0.0f, 0.0f, 0.0f};
            initializeCovariance();
            filterInitialized = true;
            pendingReinitializeAfterCalibration = false;
        }

        predictWithGyro(fusedGyro, samplePeriodS);
        debugState = 50U;
        nowUs = tap::arch::clock::getTimeMicroseconds();
        timingProfile.predictUs = nowUs - stageStartUs;
        stageStartUs = nowUs;

        for (size_t i = 0; i < N; i++)
        {
            debugState = 60U + static_cast<uint32_t>(i);
            if (!validFlags[i])
            {
                continue;
            }
            if (runAccelUpdate(accel[i], accelVar[i]))
            {
                timingProfile.updates++;
            }
        }
        nowUs = tap::arch::clock::getTimeMicroseconds();
        timingProfile.updateUs = nowUs - stageStartUs;
        stageStartUs = nowUs;

        updateEulerFromQuaternion();
        debugState = 70U;
        imuData.gyroRadPerSec = tap::algorithms::transforms::Vector(
            fusedGyro.x() - (config.enableGyroBiasEstimation ? gyroBias[0] : 0.0f),
            fusedGyro.y() - (config.enableGyroBiasEstimation ? gyroBias[1] : 0.0f),
            fusedGyro.z() - (config.enableGyroBiasEstimation ? gyroBias[2] : 0.0f));
        imuData.accG = tap::algorithms::transforms::Vector(
            fusedAccel.x() - (config.enableAccelBiasEstimation ? accelBias[0] : 0.0f),
            fusedAccel.y() - (config.enableAccelBiasEstimation ? accelBias[1] : 0.0f),
            fusedAccel.z() - (config.enableAccelBiasEstimation ? accelBias[2] : 0.0f));
        imuData.temperature = (tempCount > 0) ? (tempSum / tempCount) : 0.0f;

        nowUs = tap::arch::clock::getTimeMicroseconds();
        timingProfile.mahonyUs = nowUs - stageStartUs;
        emitTiming(cycleStartUs, nowUs);
        debugState = 99U;
    }

    inline const char* getName() const override { return "FusedIMUMEKF"; }
    inline float getAccelerationSensitivity() const override
    {
        return tap::communication::sensors::imu::GRAVITY_MPS2;
    }

    inline float getYaw() const override { return wrapAngle(yawRad); }
    inline float getPitch() const override { return pitchRad; }
    inline float getRoll() const override { return rollRad; }

    inline const TimingProfile& getTimingProfile() const { return timingProfile; }
    inline uint32_t getDebugState() const { return debugState; }
    inline void setTimingTelemetryCallback(
        TimingTelemetryCallback callback,
        uint16_t decimation = 200U)
    {
        timingTelemetryCallback = callback;
        timingTelemetryDecimation = (decimation == 0U) ? 1U : decimation;
        timingTelemetryCounter = 0U;
    }

private:
    static constexpr size_t kErrorStateSize = 9;
    static constexpr size_t kQuatSize = 4;

    Config config;
    std::array<tap::communication::sensors::imu::AbstractIMU*, N> imus;
    std::array<tap::algorithms::transforms::Transform, N> imuTransforms;
    std::array<tap::algorithms::transforms::Transform, N> imuToFusionTransforms;
    std::array<typename Config::ImuNoiseDensity, N> perImuNoise;
    std::array<std::array<float, 3>, N> accelVar{};
    std::array<std::array<float, 3>, N> gyroVar{};

    float samplePeriodS = 0.001f;
    uint32_t prevFilterUpdateTimeUs = 0U;
    bool filterInitialized = false;
    bool pendingReinitializeAfterCalibration = false;

    // Nominal state
    std::array<float, kQuatSize> q = {1.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 3> gyroBias = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> accelBias = {0.0f, 0.0f, 0.0f};

    // Error covariance P (9x9 row-major)
    std::array<float, kErrorStateSize * kErrorStateSize> P{};

    float rollRad = 0.0f;
    float pitchRad = 0.0f;
    float yawRad = 0.0f;

    TimingProfile timingProfile{};
    TimingTelemetryCallback timingTelemetryCallback = nullptr;
    uint16_t timingTelemetryDecimation = 200U;
    uint16_t timingTelemetryCounter = 0U;

    static inline float clampf(float v, float lo, float hi)
    {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

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
                const float ndAcc =
                    (perImuNoise[i].accelNoiseDensityUgSqrtHz[a] * 1.0e-6f) *
                    tap::communication::sensors::imu::GRAVITY_MPS2;
                accelVar[i][a] = ndAcc * ndAcc * bw;

                const float ndGyro =
                    modm::toRadian(perImuNoise[i].gyroNoiseDensityMdpsSqrtHz[a] * 1.0e-3f);
                gyroVar[i][a] = ndGyro * ndGyro * bw;
            }
        }
    }

    inline tap::algorithms::transforms::Vector weightedAverage(
        const std::array<tap::algorithms::transforms::Vector, N>& values,
        const std::array<bool, N>& validFlags,
        const std::array<std::array<float, 3>, N>& variances) const
    {
        float accum[3] = {0.0f, 0.0f, 0.0f};
        float wsum[3] = {0.0f, 0.0f, 0.0f};
        for (size_t i = 0; i < N; i++)
        {
            if (!validFlags[i])
            {
                continue;
            }
            const float v[3] = {values[i].x(), values[i].y(), values[i].z()};
            for (size_t a = 0; a < 3; a++)
            {
                const float var = clampf(variances[i][a], 1.0e-10f, 1.0e6f);
                const float w = 1.0f / var;
                accum[a] += v[a] * w;
                wsum[a] += w;
            }
        }

        float out[3] = {0.0f, 0.0f, 0.0f};
        for (size_t a = 0; a < 3; a++)
        {
            if (wsum[a] > 1.0e-10f)
            {
                out[a] = accum[a] / wsum[a];
            }
        }
        return tap::algorithms::transforms::Vector(out[0], out[1], out[2]);
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

        P[0 * kErrorStateSize + 0] = angleVar;
        P[1 * kErrorStateSize + 1] = angleVar;
        P[2 * kErrorStateSize + 2] = angleVar;
        P[3 * kErrorStateSize + 3] = gyroBiasVar;
        P[4 * kErrorStateSize + 4] = gyroBiasVar;
        P[5 * kErrorStateSize + 5] = gyroBiasVar;
        P[6 * kErrorStateSize + 6] = accelBiasVar;
        P[7 * kErrorStateSize + 7] = accelBiasVar;
        P[8 * kErrorStateSize + 8] = accelBiasVar;
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
        debugState = 100U;
        const float bgx = config.enableGyroBiasEstimation ? gyroBias[0] : 0.0f;
        const float bgy = config.enableGyroBiasEstimation ? gyroBias[1] : 0.0f;
        const float bgz = config.enableGyroBiasEstimation ? gyroBias[2] : 0.0f;
        float wx = gyroMeas.x() - bgx;
        float wy = gyroMeas.y() - bgy;
        float wz = gyroMeas.z() - bgz;

        const std::array<float, 4> omegaQ = {0.0f, wx, wy, wz};
        std::array<float, 4> dq = quatMul(q, omegaQ);
        debugState = 101U;
        for (size_t i = 0; i < 4; i++)
        {
            q[i] += 0.5f * dq[i] * dt;
        }
        quatNormalize(q);
        debugState = 102U;

        // Phi = I + F*dt for error-state [dtheta dbg dba]
        std::array<float, kErrorStateSize * kErrorStateSize> Phi{};
        for (size_t i = 0; i < kErrorStateSize; i++)
        {
            Phi[i * kErrorStateSize + i] = 1.0f;
        }
        debugState = 103U;

        const float dtwx = wx * dt;
        const float dtwy = wy * dt;
        const float dtwz = wz * dt;
        // -skew(w)*dt
        Phi[0 * kErrorStateSize + 1] += dtwz;
        Phi[0 * kErrorStateSize + 2] += -dtwy;
        Phi[1 * kErrorStateSize + 0] += -dtwz;
        Phi[1 * kErrorStateSize + 2] += dtwx;
        Phi[2 * kErrorStateSize + 0] += dtwy;
        Phi[2 * kErrorStateSize + 1] += -dtwx;
        // dtheta/dbg
        Phi[0 * kErrorStateSize + 3] = -dt;
        Phi[1 * kErrorStateSize + 4] = -dt;
        Phi[2 * kErrorStateSize + 5] = -dt;

        std::array<float, kErrorStateSize * kErrorStateSize> tmp{};
        std::array<float, kErrorStateSize * kErrorStateSize> pNew{};
        debugState = 104U;
        for (size_t r = 0; r < kErrorStateSize; r++)
        {
            for (size_t c = 0; c < kErrorStateSize; c++)
            {
                float s = 0.0f;
                for (size_t k = 0; k < kErrorStateSize; k++)
                {
                    s += Phi[r * kErrorStateSize + k] * P[k * kErrorStateSize + c];
                }
                tmp[r * kErrorStateSize + c] = s;
            }
        }
        debugState = 105U;
        for (size_t r = 0; r < kErrorStateSize; r++)
        {
            for (size_t c = 0; c < kErrorStateSize; c++)
            {
                float s = 0.0f;
                for (size_t k = 0; k < kErrorStateSize; k++)
                {
                    s += tmp[r * kErrorStateSize + k] * Phi[c * kErrorStateSize + k];
                }
                pNew[r * kErrorStateSize + c] = s;
            }
        }
        debugState = 106U;

        // Qd diagonal
        const float gyroVarAvg = (gyroVar[0][0] + gyroVar[0][1] + gyroVar[0][2]) * (1.0f / 3.0f);
        const float qTheta = clampf(gyroVarAvg * dt, 1.0e-12f, 1.0f);
        const float qBg = config.enableGyroBiasEstimation
                              ? (config.gyroBiasRandomWalkStdRadPerSec *
                                 config.gyroBiasRandomWalkStdRadPerSec * dt)
                              : 0.0f;
        const float qBa = config.enableAccelBiasEstimation
                              ? (config.accelBiasRandomWalkStdMps2 *
                                 config.accelBiasRandomWalkStdMps2 * dt)
                              : 0.0f;
        pNew[0 * kErrorStateSize + 0] += qTheta;
        pNew[1 * kErrorStateSize + 1] += qTheta;
        pNew[2 * kErrorStateSize + 2] += qTheta;
        pNew[3 * kErrorStateSize + 3] += qBg;
        pNew[4 * kErrorStateSize + 4] += qBg;
        pNew[5 * kErrorStateSize + 5] += qBg;
        pNew[6 * kErrorStateSize + 6] += qBa;
        pNew[7 * kErrorStateSize + 7] += qBa;
        pNew[8 * kErrorStateSize + 8] += qBa;

        P = pNew;
        debugState = 107U;
    }

    inline bool invert3x3(const float a[9], float invOut[9]) const
    {
        debugState = 200U;
        const float det = a[0] * (a[4] * a[8] - a[5] * a[7]) -
                          a[1] * (a[3] * a[8] - a[5] * a[6]) +
                          a[2] * (a[3] * a[7] - a[4] * a[6]);
        if (std::fabs(det) < 1.0e-12f)
        {
            debugState = 201U;
            return false;
        }
        const float invDet = 1.0f / det;
        debugState = 202U;
        invOut[0] = (a[4] * a[8] - a[5] * a[7]) * invDet;
        invOut[1] = (a[2] * a[7] - a[1] * a[8]) * invDet;
        invOut[2] = (a[1] * a[5] - a[2] * a[4]) * invDet;
        invOut[3] = (a[5] * a[6] - a[3] * a[8]) * invDet;
        invOut[4] = (a[0] * a[8] - a[2] * a[6]) * invDet;
        invOut[5] = (a[2] * a[3] - a[0] * a[5]) * invDet;
        invOut[6] = (a[3] * a[7] - a[4] * a[6]) * invDet;
        invOut[7] = (a[1] * a[6] - a[0] * a[7]) * invDet;
        invOut[8] = (a[0] * a[4] - a[1] * a[3]) * invDet;
        debugState = 203U;
        return true;
    }

    inline bool runAccelUpdate(
        const tap::algorithms::transforms::Vector& accelMeas,
        const std::array<float, 3>& accelVarDiag)
    {
        debugState = 300U;
        const float ax = accelMeas.x();
        const float ay = accelMeas.y();
        const float az = accelMeas.z();

        const float norm = std::sqrt(ax * ax + ay * ay + az * az);
        if (norm < config.minAccelNormMps2 || norm > config.maxAccelNormMps2)
        {
            debugState = 301U;
            return false;
        }

        const float g = tap::communication::sensors::imu::GRAVITY_MPS2;
        if (std::fabs(norm - g) > config.accelGateMps2)
        {
            debugState = 302U;
            return false;
        }

        const auto gBody = gravityBodyFromQuat();
        debugState = 303U;
        const float h[3] = {
            gBody[0] + accelBias[0],
            gBody[1] + accelBias[1],
            gBody[2] + accelBias[2]};
        const float r[3] = {ax - h[0], ay - h[1], az - h[2]};

        // H = [ skew(gBody) 0 I ]
        float H[3 * kErrorStateSize] = {};
        debugState = 304U;
        H[0 * kErrorStateSize + 1] = -gBody[2];
        H[0 * kErrorStateSize + 2] = gBody[1];
        H[1 * kErrorStateSize + 0] = gBody[2];
        H[1 * kErrorStateSize + 2] = -gBody[0];
        H[2 * kErrorStateSize + 0] = -gBody[1];
        H[2 * kErrorStateSize + 1] = gBody[0];
        if (config.enableAccelBiasEstimation)
        {
            H[0 * kErrorStateSize + 6] = 1.0f;
            H[1 * kErrorStateSize + 7] = 1.0f;
            H[2 * kErrorStateSize + 8] = 1.0f;
        }

        // PHt = P * H^T (9x3)
        float PHt[kErrorStateSize * 3] = {};
        debugState = 305U;
        for (size_t i = 0; i < kErrorStateSize; i++)
        {
            for (size_t j = 0; j < 3; j++)
            {
                float s = 0.0f;
                for (size_t k = 0; k < kErrorStateSize; k++)
                {
                    s += P[i * kErrorStateSize + k] * H[j * kErrorStateSize + k];
                }
                PHt[i * 3 + j] = s;
            }
        }
        debugState = 306U;

        // S = H*PHt + R (3x3)
        float S[9] = {};
        debugState = 307U;
        for (size_t i = 0; i < 3; i++)
        {
            for (size_t j = 0; j < 3; j++)
            {
                float s = 0.0f;
                for (size_t k = 0; k < kErrorStateSize; k++)
                {
                    s += H[i * kErrorStateSize + k] * PHt[k * 3 + j];
                }
                S[i * 3 + j] = s;
            }
        }
        debugState = 308U;
        S[0] += clampf(accelVarDiag[0] * config.accelMeasurementVarianceScale, 1.0e-8f, 1.0e5f);
        S[4] += clampf(accelVarDiag[1] * config.accelMeasurementVarianceScale, 1.0e-8f, 1.0e5f);
        S[8] += clampf(accelVarDiag[2] * config.accelMeasurementVarianceScale, 1.0e-8f, 1.0e5f);
        debugState = 309U;

        float SInv[9] = {};
        if (!invert3x3(S, SInv))
        {
            debugState = 310U;
            return false;
        }
        debugState = 311U;

        // K = PHt * SInv (9x3)
        float K[kErrorStateSize * 3] = {};
        debugState = 312U;
        for (size_t i = 0; i < kErrorStateSize; i++)
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
        debugState = 313U;

        // dx = K*r
        float dx[kErrorStateSize] = {};
        debugState = 314U;
        for (size_t i = 0; i < kErrorStateSize; i++)
        {
            dx[i] = K[i * 3 + 0] * r[0] + K[i * 3 + 1] * r[1] + K[i * 3 + 2] * r[2];
        }
        debugState = 315U;

        // State correction
        const float dthx = config.attitudeCorrectionGain * dx[0];
        const float dthy = config.attitudeCorrectionGain * dx[1];
        const float dthz = config.attitudeCorrectionGain * dx[2];
        const std::array<float, 4> dq = {1.0f, 0.5f * dthx, 0.5f * dthy, 0.5f * dthz};
        q = quatMul(q, dq);
        quatNormalize(q);
        if (config.enableGyroBiasEstimation)
        {
            gyroBias[0] += config.gyroBiasCorrectionGain * dx[3];
            gyroBias[1] += config.gyroBiasCorrectionGain * dx[4];
            gyroBias[2] += config.gyroBiasCorrectionGain * dx[5];
        }
        if (config.enableAccelBiasEstimation)
        {
            accelBias[0] += config.accelBiasCorrectionGain * dx[6];
            accelBias[1] += config.accelBiasCorrectionGain * dx[7];
            accelBias[2] += config.accelBiasCorrectionGain * dx[8];
        }
        debugState = 316U;

        // P = (I - K H) P
        std::array<float, kErrorStateSize * kErrorStateSize> IminusKH{};
        debugState = 317U;
        for (size_t i = 0; i < kErrorStateSize; i++)
        {
            IminusKH[i * kErrorStateSize + i] = 1.0f;
        }
        debugState = 318U;
        for (size_t i = 0; i < kErrorStateSize; i++)
        {
            for (size_t j = 0; j < kErrorStateSize; j++)
            {
                float s = 0.0f;
                for (size_t k = 0; k < 3; k++)
                {
                    s += K[i * 3 + k] * H[k * kErrorStateSize + j];
                }
                IminusKH[i * kErrorStateSize + j] -= s;
            }
        }
        debugState = 319U;

        std::array<float, kErrorStateSize * kErrorStateSize> pNew{};
        debugState = 320U;
        for (size_t rIdx = 0; rIdx < kErrorStateSize; rIdx++)
        {
            for (size_t cIdx = 0; cIdx < kErrorStateSize; cIdx++)
            {
                float s = 0.0f;
                for (size_t k = 0; k < kErrorStateSize; k++)
                {
                    s += IminusKH[rIdx * kErrorStateSize + k] * P[k * kErrorStateSize + cIdx];
                }
                pNew[rIdx * kErrorStateSize + cIdx] = s;
            }
        }
        debugState = 321U;

        // Keep symmetric
        for (size_t rIdx = 0; rIdx < kErrorStateSize; rIdx++)
        {
            for (size_t cIdx = rIdx + 1; cIdx < kErrorStateSize; cIdx++)
            {
                const float sym = 0.5f * (pNew[rIdx * kErrorStateSize + cIdx] +
                                          pNew[cIdx * kErrorStateSize + rIdx]);
                pNew[rIdx * kErrorStateSize + cIdx] = sym;
                pNew[cIdx * kErrorStateSize + rIdx] = sym;
            }
        }
        P = pNew;
        debugState = 322U;
        return true;
    }

    inline tap::algorithms::transforms::Vector transformAcceleration(
        const tap::algorithms::transforms::Transform& fusionToImu,
        const tap::algorithms::transforms::Transform& imuToFusion,
        const tap::algorithms::transforms::Vector& imuAcc) const
    {
        debugState = 800U;
        debugState = 802U;
        const auto imuPosition = fusionToImu.getTranslation();
        debugState = 803U;
        const auto fusionAngVel = fusionToImu.getAngularVel();
        debugState = 804U;
        const auto imuVelocity = tap::algorithms::transforms::Vector(
            fusionAngVel.y() * imuPosition.z() - fusionAngVel.z() * imuPosition.y(),
            fusionAngVel.z() * imuPosition.x() - fusionAngVel.x() * imuPosition.z(),
            fusionAngVel.x() * imuPosition.y() - fusionAngVel.y() * imuPosition.x());
        debugState = 805U;
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
        debugState = 806U;
        const auto fusedDynamics = imuToFusion.apply(imuDynamics);
        debugState = 807U;
        return fusedDynamics.getAcceleration();
    }

    inline tap::algorithms::transforms::Vector transformGyro(
        const tap::algorithms::transforms::Transform& fusionToImu,
        const tap::algorithms::transforms::Transform& imuToFusion,
        const tap::algorithms::transforms::Vector& imuGyro) const
    {
        debugState = 820U;
        debugState = 822U;
        const tap::algorithms::transforms::DynamicOrientation imuDynamics(
            fusionToImu.getRoll(),
            fusionToImu.getPitch(),
            fusionToImu.getYaw(),
            imuGyro.x(),
            imuGyro.y(),
            imuGyro.z());
        debugState = 823U;
        const auto fusedDynamics = imuToFusion.apply(imuDynamics);
        debugState = 824U;
        const auto fusedAngVel = fusedDynamics.getAngularVelocity();
        debugState = 825U;
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
               state == tap::communication::sensors::imu::ImuInterface::ImuState::IMU_NOT_CALIBRATED;
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

    inline void recomputeImuToFusionTransform(size_t index)
    {
        if (index >= N)
        {
            return;
        }
        debugState = 900U + static_cast<uint32_t>(index);
        imuToFusionTransforms[index] = mountingTransform.compose(imuTransforms[index].getInverse());
        debugState = 910U + static_cast<uint32_t>(index);
    }

    inline void recomputeImuToFusionTransforms()
    {
        for (size_t i = 0; i < N; i++)
        {
            recomputeImuToFusionTransform(i);
        }
    }

    inline void emitTiming(uint32_t cycleStartUs, uint32_t nowUs)
    {
        timingProfile.totalUs = nowUs - cycleStartUs;
        if (timingTelemetryCallback == nullptr)
        {
            return;
        }
        timingTelemetryCounter++;
        if ((timingTelemetryCounter % timingTelemetryDecimation) != 0U)
        {
            return;
        }
        timingTelemetryCallback("perf:fused_imu_mekf:total_us", timingProfile.totalUs);
        timingTelemetryCallback("perf:fused_imu_mekf:predict_us", timingProfile.predictUs);
        timingTelemetryCallback("perf:fused_imu_mekf:update_us", timingProfile.updateUs);
        timingTelemetryCallback("perf:fused_imu_mekf:collect_us", timingProfile.collectUs);
        timingTelemetryCallback("perf:fused_imu_mekf:n_updates", timingProfile.updates);
    }
};
}  // namespace aruwsrc::communication::sensors::imu

#endif  // FUSED_IMU_MEKF_HPP_
