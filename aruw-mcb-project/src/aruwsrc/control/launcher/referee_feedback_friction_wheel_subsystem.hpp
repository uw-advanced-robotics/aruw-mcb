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

#include "aruwsrc/drivers.hpp"
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
        aruwsrc::Drivers *drivers,
        tap::motor::MotorId leftMotorId,
        tap::motor::MotorId rightMotorId,
        tap::can::CanBus canBus,
        tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID,
        const float defaultFiringSpeed);

    /**
     * @return The predicted launch speed of the next projectile in m/s, using measured feedback
     * from the referee system barrel system to dynamically predict the barrel speed based on
     * previous barrel speeds.
     */
    inline float getPredictedLaunchSpeed() const override final
    {
        return ballSpeedAveragingTracker.getSize() == 0
                   ? getDesiredLaunchSpeed()
                   : (pastProjectileVelocitySpeedSummed / ballSpeedAveragingTracker.getSize());
    }

    void refresh() override;

private:
#if defined(TARGET_HERO)
    static constexpr size_t PROJECTILE_LAUNCH_AVERAGING_DEQUE_SIZE = 3;
#else
    static constexpr size_t PROJECTILE_LAUNCH_AVERAGING_DEQUE_SIZE = 10;
#endif

    const tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID;

    const float defaultFiringSpeed;

    modm::BoundedDeque<float, PROJECTILE_LAUNCH_AVERAGING_DEQUE_SIZE> ballSpeedAveragingTracker;

    float lastDesiredLaunchSpeed = 0;

    uint32_t prevLaunchingDataReceiveTimestamp = 0;

    float pastProjectileVelocitySpeedSummed = 0;

    void updatePredictedLaunchSpeed();
};
}  // namespace aruwsrc::control::launcher

#endif  // REFEREE_FEEDBACK_FRICTION_WHEEL_SUBSYSTEM_HPP_
