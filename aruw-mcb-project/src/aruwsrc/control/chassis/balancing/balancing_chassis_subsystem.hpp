/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BALANCING_CHASSIS_SUBSYSTEM_HPP_
#define BALANCING_CHASSIS_SUBSYSTEM_HPP_

#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/control/chassis/power_limiter.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"

#include "balancing_leg.hpp"

namespace aruwsrc::chassis
{
class BalancingChassisSubsystem : public tap::control::Subsystem
{
public:
    BalancingChassisSubsystem(
        tap::Drivers* drivers,
        aruwsrc::can::TurretMCBCanComm& turretMCB,
        BalancingLeg& leftLeg,
        BalancingLeg& rightLeg,
        tap::gpio::Analog::Pin currentPin = CURRENT_SENSOR_PIN);

    void initialize() override;

    void refresh() override;

    void runHardwareTests() override;

    const char* getName() override { return "Balancing Chassis Subsystem"; }

    void setDesiredHeight(float z)
    {
        desiredZ = tap::algorithms::limitVal<float>(desiredZ + z, -.35, -.15);
    };

    /**
     * @brief Defines x position and r rotation angle that we want the chassis to be in
     *
     * @param v: (m/s) Velocity Setpoint Relative to current position
     * @param r: (rad) Yaw Setpoint relative to current Yaw
     */
    void setDesiredOutput(float v, float r)
    {
        desiredV = v;
        desiredR = r;
    };

    void limitChassisPower();

    static inline float getMaxWheelSpeed(bool refSerialOnline, int chassisPower)
    {
        if (!refSerialOnline)
        {
            chassisPower = 0;
        }

        // only re-interpolate when needed (since this function is called a lot and the chassis
        // power rarely changes, this helps cut down on unnecessary array searching/interpolation)
        if (lastComputedMaxWheelSpeed.first != chassisPower)
        {
            lastComputedMaxWheelSpeed.first = chassisPower;
            lastComputedMaxWheelSpeed.second =
                CHASSIS_POWER_TO_SPEED_INTERPOLATOR.interpolate(chassisPower);
        }

        return lastComputedMaxWheelSpeed.second;
    }

    tap::algorithms::SmoothPid rotationPid;

private:
    void computeState();

    aruwsrc::can::TurretMCBCanComm& turretMCB;

    tap::communication::sensors::current::AnalogCurrentSensor currentSensor;

    tap::control::chassis::PowerLimiter chassisPowerLimiter;
    BalancingLeg& leftLeg, rightLeg;

    static modm::Pair<int, float> lastComputedMaxWheelSpeed;

    float pitchAdjustment = 0;
    float pitchAdjustmentPrev = 0;
    float velocityAdjustment = 0;
    float velocityAdjustmentPrev = 0;
    float targetPitch;

    float pitch;
    float roll;
    float yaw;
    float yawPrev;
    float yawRate;

    float desiredX, desiredV, desiredR, desiredZ;
    float currentX, currentV, currentR, currentZ;
    float prevX,    prevV,    prevR,    prevZ;
    float prevXdesired, prevVdesired;
    uint32_t prevTime;
};
}  // namespace aruwsrc::chassis

#endif  // BALANCING_CHASSIS_SUBSYSTEM_HPP_
