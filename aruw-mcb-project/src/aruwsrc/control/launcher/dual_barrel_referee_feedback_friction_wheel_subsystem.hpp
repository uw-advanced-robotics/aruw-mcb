/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef DUAL_BARREL_REFEREE_FEEDBACK_FRICTION_WHEEL_SUBSYSTEM_HPP_
#define DUAL_BARREL_REFEREE_FEEDBACK_FRICTION_WHEEL_SUBSYSTEM_HPP_

#include "tap/drivers.hpp"

#include "referee_feedback_friction_wheel_subsystem.hpp"

namespace aruwsrc::control::launcher
{
/**
 * An extension of the `RefereeFeedbackFrictionWheelSubsystem` for the dual barrel subsystem,
 * using referee system feedback to predict the launch velocity of the projectile.
 *
 * @tparam PROJECTILE_LAUNCH_AVERAGING_DEQUE_SIZE Number of balls to average when estimating the
 * next projectile velocity.
 */
template <size_t PROJECTILE_LAUNCH_AVERAGING_DEQUE_SIZE>
class DualBarrelRefereeFeedbackFrictionWheelSubsystem
    : public RefereeFeedbackFrictionWheelSubsystem<PROJECTILE_LAUNCH_AVERAGING_DEQUE_SIZE>
{
public:
    using BarrelMechanismID = tap::communication::serial::RefSerialData::Rx::MechanismID;

    /**
     * For all params but `firingSystemMechanismIdLeft` and `firingSystemMechanismIdRight`
     * see the `FrictionWheelSubsystem`.
     * @param[in] firingSystemMechanismIdLeft The barrel ID associated with the left barrel of
     * this friction wheel subsystem.
     * @param[in] firingSystemMechanismIdRight The barrel ID associated with the right barrel of
     *  this friction wheel subsystem.
     * @param[in] bulletSpeedLowPassAlpha The low pass alpha used to combine previous and new
     * projectle speed when computing a new predicted launch speed.
     */
    DualBarrelRefereeFeedbackFrictionWheelSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorId leftMotorId,
        tap::motor::MotorId rightMotorId,
        tap::can::CanBus canBus,
        aruwsrc::can::TurretMCBCanComm *turretMCB)
        : RefereeFeedbackFrictionWheelSubsystem<PROJECTILE_LAUNCH_AVERAGING_DEQUE_SIZE>(
              drivers,
              leftMotorId,
              rightMotorId,
              canBus,
              turretMCB,
              BarrelMechanismID::TURRET_17MM_1)  // placeholder
    {
    }

private:
    bool newFiringDataReceived() override
    {
        const auto &turretData = this->drivers->refSerial.getRobotData().turret;
        return this->prevLaunchingDataReceiveTimestamp !=
                   turretData.lastReceivedLaunchingInfoTimestamp &&
               (turretData.launchMechanismID == BarrelMechanismID::TURRET_17MM_1 ||
                turretData.launchMechanismID == BarrelMechanismID::TURRET_17MM_2);
    }
};
}  // namespace aruwsrc::control::launcher

#endif  // DUAL_BARREL_REFEREE_FEEDBACK_FRICTION_WHEEL_SUBSYSTEM_HPP_
