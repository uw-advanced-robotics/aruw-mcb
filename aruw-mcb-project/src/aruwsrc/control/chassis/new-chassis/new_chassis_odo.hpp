/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CHASSIS_ODOMETRY_HPP_
#define CHASSIS_ODOMETRY_HPP_

#include "tap/algorithms/kalman_filter.hpp"
#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"
#include "tap/algorithms/odometry/chassis_world_yaw_observer_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "modm/math/geometry/location_2d.hpp"
#include "modm/math/interpolation/linear.hpp"

#include "wheel.hpp"
namespace aruwsrc
{
namespace chassis
{
template <uint8_t numSwerve, uint8_t numOther>
class ChassisOdometry : public tap::algorithms::odometry::Odometry2DInterface
{
public:
    ChassisOdometry(
        tap::Drivers* drivers,
        std::vector<Wheel*>& wheels,
        tap::communication::sensors::imu::ImuInterface& imu,
        const modm::Vector2f& initPos,
        const float (&cMat)[6 * (numSwerve * 2 + numOther)],
        const float (&qMat)[6 * 6],
        const float (&rMat)[(numSwerve * 2 + numOther) * (numSwerve * 2 + numOther)],
        const tap::algorithms::odometry::ChassisWorldYawObserverInterface* chassisYawObserver)
        : wheels(wheels),
          chassisYawObserver(chassisYawObserver),
          imu(imu),
          initPos(initPos),
          kf(KF_A, cMat, qMat, rMat, KF_P0)
    {
        reset();
    }
    ChassisOdometry(
        tap::Drivers* drivers,
        std::vector<Wheel*>& wheels,
        tap::communication::sensors::imu::ImuInterface& imu,
        const modm::Vector2f& initPos,
        const float (&cMat)[6 * (numSwerve * 2 + numOther)],
        const float (&qMat)[6 * 6],
        const float (&rMat)[(numSwerve * 2 + numOther) * (numSwerve * 2 + numOther)])
        : wheels(wheels),
          chassisYawObserver(nullptr),
          imu(imu),
          initPos(initPos),
          kf(KF_A, cMat, qMat, rMat, KF_P0)
    {
        reset();
    }

    // void refresh() { update(); }

    class ChassisOdometryBuilder
    {
    public:
        static ChassisOdometry constructChassisOdometry(
            std::vector<Wheel*>& wheels,
            tap::Drivers& drivers,
            const modm::Vector2f initPos,
            const tap::algorithms::odometry::ChassisWorldYawObserverInterface* chassisYawObserver)
        {
            std::vector<float> CMat;
            std::vector<float> QMat;
            std::vector<float> P0Mat;
            std::vector<float> RMat;
            int totalSize = numSwerve * 2 + numOther;
            for (const auto& wheel : wheels)
            {
                CMat.insert(CMat.end(), wheel->getHMat().begin(), wheel->getHMat().end());
            }
            for (int i = 0; i < totalSize * totalSize; ++i)
            {
                int row = i / totalSize;
                int col = i % totalSize;
                if (row == col)
                {
                    RMat[i] = wheels[i]->config.rConfidence;
                }
                else
                {
                    RMat[i] = 0.0;
                }
            }
            for (int i = 0; i < totalSize * totalSize; ++i)
            {
                int row = i / totalSize;
                int col = i % totalSize;
                if (row == col)
                {
                    QMat[i] = wheels[i]->config.initalQValue;
                }
                else
                {
                    QMat[i] = 0.0;
                }
            }

            float(cMatArray)[6 * (numSwerve * 2 + numOther)];
            float(qMatArray)[6 * 6];
            float(rMatArray)[(numSwerve * 2 + numOther) * (numSwerve * 2 + numOther)];
            for (u_int8_t i = 0; i < CMat.size(); ++i)
            {
                cMatArray[i] = CMat[i];
            }
            for (u_int8_t i = 0; i < QMat.size(); ++i)
            {
                qMatArray[i] = QMat[i];
            }
            for (u_int8_t i = 0; i < RMat.size(); ++i)
            {
                rMatArray[i] = RMat[i];
            }

            return ChassisOdometry<numSwerve, numOther>(
                &drivers,
                wheels,
                drivers.mpu6500,
                initPos,
                cMatArray,
                qMatArray,
                rMatArray,
                chassisYawObserver);
        }
        static ChassisOdometry constructChassisOdometry(
            std::vector<Wheel*>& wheels,
            tap::Drivers& drivers,
            const modm::Vector2f initPos)
        {
            std::vector<float> CMat;
            std::vector<float> QMat;
            std::vector<float> P0Mat;
            std::vector<float> RMat;
            int totalSize = numSwerve * 2 + numOther;
            for (const auto& wheel : wheels)
            {
                CMat.insert(CMat.end(), wheel->getHMat().begin(), wheel->getHMat().end());
            }
            for (int i = 0; i < totalSize * totalSize; ++i)
            {
                int row = i / totalSize;
                int col = i % totalSize;
                if (row == col)
                {
                    RMat[i] = wheels[i]->config.rConfidence;
                }
                else
                {
                    RMat[i] = 0.0;
                }
            }
            for (int i = 0; i < totalSize * totalSize; ++i)
            {
                int row = i / totalSize;
                int col = i % totalSize;
                if (row == col)
                {
                    QMat[i] = wheels[i]->config.initalQValue;
                }
                else
                {
                    QMat[i] = 0.0;
                }
            }

            float(cMatArray)[6 * (numSwerve * 2 + numOther)];
            float(qMatArray)[6 * 6];
            float(rMatArray)[(numSwerve * 2 + numOther) * (numSwerve * 2 + numOther)];
            for (u_int8_t i = 0; i < CMat.size(); ++i)
            {
                cMatArray[i] = CMat[i];
            }
            for (u_int8_t i = 0; i < QMat.size(); ++i)
            {
                qMatArray[i] = QMat[i];
            }
            for (u_int8_t i = 0; i < RMat.size(); ++i)
            {
                rMatArray[i] = RMat[i];
            }

            return ChassisOdometry<numSwerve, numOther>(
                &drivers,
                wheels,
                drivers.mpu6500,
                initPos,
                cMatArray,
                qMatArray,
                rMatArray);
        }
    };

    inline modm::Location2D<float> getCurrentLocation2D() const final { return location; }

    inline modm::Vector2f getCurrentVelocity2D() const final { return velocity; }

    inline uint32_t getLastComputedOdometryTime() const final { return prevTime; }

    inline float getYaw() const override { return chassisYaw; }

    /**
     * @brief Resets the KF back to the robot's boot position.
     */

    void reset()
    {
        float initialX[int(OdomState::NUM_STATES)] = {initPos.x, 0.0f, initPos.y, 0.0f, 0.0f};
        kf.init(initialX);
    }
    void update()
    {
        if (chassisYawObserver != nullptr && !chassisYawObserver->getChassisWorldYaw(&chassisYaw))
        {
            chassisYaw = 0;
            // return;
        }

        float z[(numSwerve * 2) + numOther] = {};

        for (int i = 0; i < ((numSwerve * 2) + numOther); ++i)
        {
            for (auto entry : wheels[i]->getMMat())
            {
                z[i] = entry;
                if (wheels[i]->getMMat().size() > 1)
                {
                    i++;
                }
            }
        }

        // perform the update, after this update a new state matrix is now available
        kf.performUpdate(z);

        updateChassisStateFromKF(chassisYaw);
    }

    void updateChassisStateFromKF(float chassisYaw)
    {
        const auto& x = kf.getStateVectorAsMatrix();

        location.setOrientation(chassisYaw);
        location.setPosition(x[int(OdomState::POS_X)], x[int(OdomState::POS_Y)]);
    }

private:
    enum class OdomState
    {
        POS_X = 0,
        POS_Y,
        POS_THETA,
        VEL_X,
        VEL_Y,
        VEL_THETA,
        NUM_STATES,
    };

    enum class OdomInput
    {
        VEL_X = 0,
        VEL_Y,
        VEL_Z,
        VEL_THETA,
        NUM_INPUTS,
    };

    static constexpr int STATES_SQUARED =
        static_cast<int>(OdomState::NUM_STATES) * static_cast<int>(OdomState::NUM_STATES);
    static constexpr int INPUTS_SQUARED =
        static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomInput::NUM_INPUTS);
    static constexpr int INPUTS_MULT_STATES =
        static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomState::NUM_STATES);

    /// Assumed time difference between calls to `update`, in seconds
    static constexpr float DT = 0.002f;

    // clang-format off
    // 6 by 6
    static constexpr float KF_A[STATES_SQUARED] = {
        1, DT, 0 , 0 , 0 , 0 ,
        0, 0 , 1 , DT, 0 , 0 ,
        0, 0 , 0 , 0 , 1 , DT,
        0, 1 , 0 , 0 , 0 , 0 ,
        0, 0 , 0 , 1 , 0 , 0 ,
        0, 0 , 0 , 0 , 0 , 1 ,
    };
    static constexpr float KF_Q[STATES_SQUARED] = {
        1E2, 0  , 0  , 0  , 0  , 0  ,
        0  , 1E1, 0  , 0  , 0  , 0  ,
        0  , 0  , 5E0, 0  , 0  , 0  ,
        0  , 0  , 0  , 1E2, 0  , 0  ,
        0  , 0  , 0  , 0  , 1E1, 0  ,
        0  , 0  , 0  , 0  , 0  , 5E0,
    };
    static constexpr float KF_P0[STATES_SQUARED] = {
        1E3, 0  , 0  , 0  , 0  , 0  ,
        0  , 1E3, 0  , 0  , 0  , 0  ,
        0  , 0  , 1E3, 0  , 0  , 0  ,
        0  , 0  , 0  , 1E3, 0  , 0  ,
        0  , 0  , 0  , 0  , 1E3, 0  ,
        0  , 0  , 0  , 0  , 0  , 1E3,
    };
    static float KF_C[INPUTS_MULT_STATES];
    static float KF_R[INPUTS_SQUARED];
    // clang-format on

    /// Max chassis acceleration magnitude measured on the standard when at 120W power mode, in
    /// m/s^2. Also works for hero since it has an acceleration on the same order of magnitude.
    static constexpr float MAX_ACCELERATION = 8.0f;

    static constexpr modm::Pair<float, float> CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT[] =
        {{0, 1E0}, {MAX_ACCELERATION, 1E2}};

    static constexpr float CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA = 0.01f;

    tap::algorithms::odometry::ChassisWorldYawObserverInterface* chassisYawObserver;
    tap::communication::sensors::imu::ImuInterface& imu;
    std::vector<Wheel*>& wheels;

    const modm::Vector2f initPos;

    tap::algorithms::KalmanFilter<int(OdomState::NUM_STATES), int(numSwerve * 2 + numOther)> kf;

    /// Chassis location in the world frame
    modm::Location2D<float> location;
    /// Chassis velocity in the world frame
    modm::Vector2f velocity;
    // Chassis yaw orientation in world frame (radians)
    float chassisYaw = 0;

    /// Previous time `update` was called, in microseconds
    uint32_t prevTime = 0;
    modm::Matrix<float, 3, 1> prevChassisVelocity;

    void updateMeasurementCovariance(const modm::Matrix<float, 3, 1>& chassisVelocity);
};
}  // namespace chassis
}  // namespace aruwsrc

#endif  // CHASSIS_KF_ODOMETRY_HPP_
