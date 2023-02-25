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

#ifndef REFEREE_FEEDBACK_FRICTION_WHEEL_SUBSYSTEM_HPP_
#define REFEREE_FEEDBACK_FRICTION_WHEEL_SUBSYSTEM_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/util_macros.hpp"

#include "tap/drivers.hpp"
#include "modm/container/deque.hpp"

#include "friction_wheel_subsystem.hpp"
#include "launch_speed_predictor_interface.hpp"

namespace aruwsrc::control::launcher
{
/**
 * An extension of the `FrictionWheelSubsystem` that implements the `LaunchSpeedPredictorInterface`,
 * using referee system feedback to predict the launch velocity of the projectile.
 *
 * @tparam PROJECTILE_LAUNCH_AVERAGING_DEQUE_SIZE Number of balls to average when estimating the
 * next projectile velocity.
 */
template <size_t PROJECTILE_LAUNCH_AVERAGING_DEQUE_SIZE>
class RefereeFeedbackFrictionWheelSubsystem : public FrictionWheelSubsystem,
                                              public LaunchSpeedPredictorInterface
{
public:
    /**
     * For all params but `firingSystemMechanismId` see the `FrictionWheelSubsystem`.
     * @param[in] firingSystemMechanismId The barrel ID associated with this friction wheel
     * subsystem.
     * @param[in] bulletSpeedLowPassAlpha The low pass alpha used to combine previous and new
     * projectle speed when computing a new predicted launch speed.
     */
    RefereeFeedbackFrictionWheelSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorId leftMotorId,
        tap::motor::MotorId rightMotorId,
        tap::can::CanBus canBus,
        aruwsrc::can::TurretMCBCanComm *turretMCB,
        tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID)
        : FrictionWheelSubsystem(drivers, leftMotorId, rightMotorId, canBus, turretMCB),
          firingSystemMechanismID(firingSystemMechanismID)
    {
    }

    /**
     * @return The predicted launch speed of the next projectile in m/s, using measured feedback
     * from the referee system barrel system to dynamically predict the barrel speed based on
     * previous barrel speeds.
     */
    inline float getPredictedLaunchSpeed() const override final_mockable
    {
        return ballSpeedAveragingTracker.getSize() == 0
                   ? getDesiredLaunchSpeed()
                   : (pastProjectileVelocitySpeedSummed / ballSpeedAveragingTracker.getSize());
    }

    void refresh() override
    {
        FrictionWheelSubsystem::refresh();
        updatePredictedLaunchSpeed();
    }

private:
    const tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID;

    modm::BoundedDeque<float, PROJECTILE_LAUNCH_AVERAGING_DEQUE_SIZE> ballSpeedAveragingTracker;

    float lastDesiredLaunchSpeed = 0;

    uint32_t prevLaunchingDataReceiveTimestamp = 0;

    float pastProjectileVelocitySpeedSummed = 0;

    void updatePredictedLaunchSpeed()
    {
        const float desiredLaunchSpeed = getDesiredLaunchSpeed();

        // reset averaging if desired launch speed has changed...if we change desired launch speed
        // from 15 to 30, we should predict the launch speed to be around 30, not 15.
        if (!tap::algorithms::compareFloatClose(lastDesiredLaunchSpeed, desiredLaunchSpeed, 1E-5))
        {
            lastDesiredLaunchSpeed = desiredLaunchSpeed;
            pastProjectileVelocitySpeedSummed = 0;
            ballSpeedAveragingTracker.clear();
        }

        if (drivers->refSerial.getRefSerialReceivingData())
        {
            const auto &turretData = drivers->refSerial.getRobotData().turret;

            // compute average bullet speed if new firing data received from correct mech ID
            if (prevLaunchingDataReceiveTimestamp !=
                    turretData.lastReceivedLaunchingInfoTimestamp &&
                turretData.launchMechanismID == firingSystemMechanismID)
            {
                // remove element to make room for new element
                if (ballSpeedAveragingTracker.isFull())
                {
                    pastProjectileVelocitySpeedSummed -= ballSpeedAveragingTracker.getFront();
                    ballSpeedAveragingTracker.removeFront();
                }

                const float limitedProjectileSpeed = tap::algorithms::limitVal(
                    turretData.bulletSpeed,
                    0.0f,
                    MAX_MEASURED_LAUNCH_SPEED);

                // insert new element
                pastProjectileVelocitySpeedSummed += limitedProjectileSpeed;
                ballSpeedAveragingTracker.append(limitedProjectileSpeed);

                prevLaunchingDataReceiveTimestamp = turretData.lastReceivedLaunchingInfoTimestamp;
            }
        }
        else
        {
            pastProjectileVelocitySpeedSummed = 0;
            ballSpeedAveragingTracker.clear();
        }
    }
};
}  // namespace aruwsrc::control::launcher

#endif  // REFEREE_FEEDBACK_FRICTION_WHEEL_SUBSYSTEM_HPP_
