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

#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"

#include "wheel.hpp"
namespace aruwsrc
{
namespace chassis
{
template <uint8_t numSwerve, uint8_t numOther>
class ChassisOdometry : public tap::algorithms::odometry::Odometry2DInterface
{
public:
    float numbersTest[10];
    ChassisOdometry(
        tap::Drivers* drivers,
        std::vector<Wheel*>& wheels,
        tap::communication::sensors::imu::ImuInterface& imu,
        const modm::Vector2f& initPos,
        const float (&cMat)[6 * (numSwerve * 2 + numOther)],
        const float (&qMat)[6 * 6],
        const float (&rMat)[(numSwerve * 2 + numOther) * (numSwerve * 2 + numOther)],
        aruwsrc::control::turret::TurretSubsystem& turret)
        : wheels(wheels),
          imu(imu),
          initPos(initPos),
          kf(KF_A, KF_C, KF_Q, KF_R, KF_P0),
          orientationObserver(turret),
          chassisYawObserver(&orientationObserver),
          chassisAccelerationToMeasurementCovarianceInterpolator(
            CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT,
            MODM_ARRAY_SIZE(CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT))
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
            aruwsrc::control::turret::TurretSubsystem& turret
)
        {
            std::vector<float> CMat;
            float(qMatArray)[6 * 6] ;
    //= {
    //     1E2, 0  , 0  , 0  , 0  , 0  ,
    //     0  , 1E1, 0  , 0  , 0  , 0  ,
    //     0  , 0  , 5E0, 0  , 0  , 0  ,
    //     0  , 0  , 0  , 1E2, 0  , 0  ,
    //     0  , 0  , 0  , 0  , 1E1, 0  ,
    //     0  , 0  , 0  , 0  , 0  , 5E0,
    // };
            float(rMatArray)[(numSwerve * 2 + numOther) * (numSwerve * 2 + numOther)];
            int totalSize = numSwerve * 2 + numOther;
            // int totalSize = 4;
            for (const auto& wheel : wheels)
            {
                CMat.insert(CMat.end(), wheel->getHMat().begin(), wheel->getHMat().end());
            }
            // for (int i = 0; i < totalSize * totalSize; i++)
            // {
            //     int row = i / totalSize;
            //     int col = i % totalSize;
            //     if (row == col)
            //     {
            //         rMatArray[i] = wheels[i]->config.rConfidence;
            //         // RMat[i] = 1.0;
            //     }
            //     else
            //     {
            //         rMatArray[i] = 30.0;     
            //     }
            // }

            

            for (int i = 0; i < totalSize; i++)
            {
                for (int j = 0; j < totalSize; j++)
                {
                    if (i == j)
                    {
                        rMatArray[i * totalSize + j] = wheels[i]->config.rConfidence;
                    }
                    else
                    {
                        rMatArray[i * totalSize + j] = 0.0;
                    }
                }
            }

            // for (int i = 0; i < (6*6); i++){
            //     qMatArray[i] = 1000.0;
            // }

            float(cMatArray)[6 * (numSwerve * 2 + numOther)];
            // float(qMatArray)[6 * 6];
            // float(rMatArray)[(numSwerve * 2 + numOther) * (numSwerve * 2 + numOther)];
            for (u_int8_t i = 0; i < CMat.size(); i++)
            {
                cMatArray[i] = CMat[i];
            }
            // for (u_int8_t i = 0; i < QMat.size(); i++)
            // {
            //     qMatArray[i] = QMat[i];
            // }
            // for (u_int8_t i = 0; i < RMat.size(); i++)
            // {
            //     rMatArray[i] = RMat[i];
            // }

            return ChassisOdometry<numSwerve, numOther>(
                &drivers,
                wheels,
                drivers.mpu6500,
                initPos,
                cMatArray,
                qMatArray,
                rMatArray,
                turret);
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
        float initialX[int(OdomState::NUM_STATES)] = {initPos.x, 0.0f, initPos.y, 0.0f, 0.0f, 0.0f};
        kf.init(initialX);
    }
    void update()
    {
        numbersTest[0] = 99;


        if (chassisYawObserver != nullptr && !chassisYawObserver->getChassisWorldYaw(&chassisYaw))
        {
            chassisYaw = 0;
            // return;
        }

        float z[(numSwerve * 2) + numOther] = {};

        for (int i = 0; i < ((numSwerve * 2) + numOther); i++)
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
        const std::array<float, 6> x = kf.getStateVectorAsMatrix();

        location.setOrientation(10);
        location.setPosition(numbersTest[0] * 10, 10);
    }

private:
         enum class OdomState
    {
        POS_X = 0,
        VEL_X,
        ACC_X,
        POS_Y,
        VEL_Y,
        ACC_Y,
        NUM_STATES,
    };

    enum class OdomInput
    {
        VEL_X = 0,
        ACC_X,
        VEL_Y,
        ACC_Y,
        NUM_INPUTS,
    };

    tap::algorithms::KalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)> kf;

private:
    static constexpr int STATES_SQUARED =
        static_cast<int>(OdomState::NUM_STATES) * static_cast<int>(OdomState::NUM_STATES);
    static constexpr int INPUTS_SQUARED =
        static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomInput::NUM_INPUTS);
    static constexpr int INPUTS_MULT_STATES =
        static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomState::NUM_STATES);

    /// Assumed time difference between calls to `update`, in seconds
    static constexpr float DT = 0.002f;

    // clang-format off
    static constexpr float KF_A[STATES_SQUARED] = {
        1, DT, 0.5 * DT * DT, 0, 0 , 0            ,
        0, 1 , DT           , 0, 0 , 0            ,
        0, 0 , 1            , 0, 0 , 0            ,
        0, 0 , 0            , 1, DT, 0.5 * DT * DT,
        0, 0 , 0            , 0, 1 , DT           ,
        0, 0 , 0            , 0, 0 , 1            ,
    };
    static constexpr float KF_C[INPUTS_MULT_STATES] = {
        1,1,1,1,1,1,
        1,1,1,1,1,1,
        1,1,1,1,1,1,
        1,1,1,1,1,1,
    };
    static constexpr float KF_Q[STATES_SQUARED] = {
        1E2, 0  , 0  , 0  , 0  , 0  ,
        0  , 1E1, 0  , 0  , 0  , 0  ,
        0  , 0  , 5E0, 0  , 0  , 0  ,
        0  , 0  , 0  , 1E2, 0  , 0  ,
        0  , 0  , 0  , 0  , 1E1, 0  ,
        0  , 0  , 0  , 0  , 0  , 5E0,
    };
    static constexpr float KF_R[INPUTS_SQUARED] = {
        1.0, 0  , 0  , 0  ,
        0  , 1.2, 0  , 0  ,
        0  , 0  , 1.0, 0  ,
        0  , 0  , 0  , 1.2,
    };
    static constexpr float KF_P0[STATES_SQUARED] = {
        1E3, 0  , 0  , 0  , 0  , 0  ,
        0  , 1E3, 0  , 0  , 0  , 0  ,
        0  , 0  , 1E3, 0  , 0  , 0  ,
        0  , 0  , 0  , 1E3, 0  , 0  ,
        0  , 0  , 0  , 0  , 1E3, 0  ,
        0  , 0  , 0  , 0  , 0  , 1E3,
    };
    // clang-format on

    /// Chassis measured change in velocity since the last time `update` was called, in the chassis
    /// frame
    modm::Vector2f chassisMeasuredDeltaVelocity;

    modm::interpolation::Linear<modm::Pair<float, float>>
        chassisAccelerationToMeasurementCovarianceInterpolator;


    // static constexpr int STATES_SQUARED =
    //     static_cast<int>(OdomState::NUM_STATES) * static_cast<int>(OdomState::NUM_STATES);
    // static constexpr int INPUTS_SQUARED = 16;
    //     // static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomInput::NUM_INPUTS);
    // static constexpr int INPUTS_MULT_STATES =
    //     // static_cast<int>(OdomInput::NUM_INPUTS) 
    //     4 * static_cast<int>(OdomState::NUM_STATES);

    /// Assumed time difference between calls to `update`, in seconds
    // static constexpr float DT = 0.002f;

    // // clang-format off
    // // 6 by 6
    // static constexpr float KF_A[STATES_SQUARED] = {
    //     1, DT, 0 , 0 , 0 , 0 ,
    //     0, 0 , 1 , DT, 0 , 0 ,
    //     0, 0 , 0 , 0 , 1 , DT,
    //     0, 1 , 0 , 0 , 0 , 0 ,
    //     0, 0 , 0 , 1 , 0 , 0 ,
    //     0, 0 , 0 , 0 , 0 , 1 ,
    // };
    // float KF_Q[6*6] = {
    //     1E2, 0  , 0  , 0  , 0  , 0  ,
    //     0  , 1E1, 0  , 0  , 0  , 0  ,
    //     0  , 0  , 5E0, 0  , 0  , 0  ,
    //     0  , 0  , 0  , 1E2, 0  , 0  ,
    //     0  , 0  , 0  , 0  , 1E1, 0  ,
    //     0  , 0  , 0  , 0  , 0  , 5E0,
    // };
    // static constexpr float KF_R[INPUTS_SQUARED] = {
    //     1.0, 0  , 0  , 0  ,
    //     0  , 1.2, 0  , 0  ,
    //     0  , 0  , 1.0, 0  ,
    //     0  , 0  , 0  , 1.2,
    // };
    // // float KF_Q[3*3] = {
    // //     1E2, 0  , 0  ,
    // //     0  , 1E1, 0  ,
    // //     0  , 0  , 5E0,
    // // };
    // static constexpr float KF_P0[STATES_SQUARED] = {
    //     1E3, 0  , 0  , 0  , 0  , 0  ,
    //     0  , 1E3, 0  , 0  , 0  , 0  ,
    //     0  , 0  , 1E3, 0  , 0  , 0  ,
    //     0  , 0  , 0  , 1E3, 0  , 0  ,
    //     0  , 0  , 0  , 0  , 1E3, 0  ,
    //     0  , 0  , 0  , 0  , 0  , 1E3,
    // };
    // static float KF_C[INPUTS_MULT_STATES];
    // // static constexpr float KF_R[INPUTS_SQUARED] = {
    // //     1,1,1,1,
    // //     1,1,1,1,
    // //     1,1,1,1,
    // //     1,1,1,1,
    // // };
    // // clang-format on

    /// Max chassis acceleration magnitude measured on the standard when at 120W power mode, in
    /// m/s^2. Also works for hero since it has an acceleration on the same order of magnitude.
    static constexpr float MAX_ACCELERATION = 8.0f;

    static constexpr modm::Pair<float, float> CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT[] =
        {{0, 1E0}, {MAX_ACCELERATION, 1E2}};

    static constexpr float CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA = 0.01f;

    aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver orientationObserver;
    tap::algorithms::odometry::ChassisWorldYawObserverInterface* chassisYawObserver;
    tap::communication::sensors::imu::ImuInterface& imu;
    std::vector<Wheel*>& wheels;

    const modm::Vector2f initPos;

    // tap::algorithms::KalmanFilter<int(OdomState::NUM_STATES), int(numSwerve * 2 + numOther)> kf;

    /// Chassis location in the world frame
    modm::Location2D<float> location;
    /// Chassis velocity in the world frame
    modm::Vector2f velocity;
    // Chassis yaw orientation in world frame (radians)
    float chassisYaw = 0;

    /// Previous time `update` was called, in microseconds
    uint32_t prevTime = 0;
    modm::Matrix<float, 3, 1> prevChassisVelocity;

    void updateMeasurementCovariance(
        const modm::Matrix<float, 3, 1>& chassisVelocity)
    {
        const uint32_t curTime = tap::arch::clock::getTimeMicroseconds();
        const uint32_t dt = curTime - prevTime;
        prevTime = curTime;

        // return to avoid weird acceleration spike on startup
        if (prevTime == 0)
        {
            return;
        }

        // compute acceleration

        chassisMeasuredDeltaVelocity.x = tap::algorithms::lowPassFilter(
            chassisMeasuredDeltaVelocity.x,
            chassisVelocity[0][0] - prevChassisVelocity[0][0],
            CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA);

        chassisMeasuredDeltaVelocity.y = tap::algorithms::lowPassFilter(
            chassisMeasuredDeltaVelocity.y,
            chassisVelocity[1][0] - prevChassisVelocity[1][0],
            CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA);

        prevChassisVelocity = chassisVelocity;

        // dt is in microseconds, acceleration is dv / dt, so to get an acceleration with units
        // m/s^2, convert dt in microseconds to seconds
        const float accelMagnitude =
            chassisMeasuredDeltaVelocity.getLength() * 1E6 / static_cast<float>(dt);

        const float velocityCovariance =
            chassisAccelerationToMeasurementCovarianceInterpolator.interpolate(accelMagnitude);

        // set measurement covariance of chassis velocity as measured by the wheels because if
        // acceleration is large, the likelihood of slippage is greater
        kf.getMeasurementCovariance()[0] = velocityCovariance;
        kf.getMeasurementCovariance()[2 * static_cast<int>(OdomInput::NUM_INPUTS) + 2] =
            velocityCovariance;
    }
};
}  // namespace chassis
}  // namespace aruwsrc

#endif  // CHASSIS_KF_ODOMETRY_HPP_
