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

#ifndef HOLONOMIC_CHASSIS_SUBSYSTEM_HPP_
#define HOLONOMIC_CHASSIS_SUBSYSTEM_HPP_

#include "tap/algorithms/extended_kalman.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/gpio/analog.hpp"
#include "tap/communication/sensors/current/current_sensor_interface.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/control/chassis/power_limiter.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/util_macros.hpp"
#include "constants/chassis_constants.hpp"
#include "modm/math/filter/pid.hpp"
#include "modm/math/matrix.hpp"

#include "capacitor_bank_power_limiter.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

namespace aruwsrc
{
namespace chassis
{
/**
 * Abstract subsystem for a holonomic chassis
 *
 * The chassis is in a right handed coordinate system with the x coordinate pointing torwards the
 * front of the chassis. As such, when looking down at the robot from above, the positive y
 * coordinate is to the left of the robot, and positive z is up. Also, the chassis rotation is
 * positive when rotating counterclockwise around the z axis.
 */
class HolonomicChassisSubsystem : public tap::control::chassis::ChassisSubsystemInterface
{
public:
    HolonomicChassisSubsystem(
        tap::Drivers* drivers,
        tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
        can::capbank::CapacitorBank* capacitorBank = nullptr);

    /**
     * Used to index into matrices returned by functions of the form get*Velocity*().
     */
    enum ChassisVelIndex
    {
        X = 0,
        Y = 1,
        R = 2,
    };

    static inline float getMaxWheelSpeed(bool refSerialOnline, float chassisPowerLimit)
    {
        if (!refSerialOnline)
        {
            chassisPowerLimit = 0;
        }

        // only re-interpolate when needed (since this function is called a lot and the chassis
        // power limit rarely changes, this helps cut down on unnecessary array
        // searching/interpolation)
        if (lastComputedMaxWheelSpeed.first != (int)chassisPowerLimit)
        {
            lastComputedMaxWheelSpeed.first = (int)chassisPowerLimit;
            lastComputedMaxWheelSpeed.second =
                CHASSIS_POWER_TO_SPEED_INTERPOLATOR.interpolate(chassisPowerLimit);
        }

        return lastComputedMaxWheelSpeed.second;
    }

    static inline float getChassisPowerLimit(tap::Drivers* drivers)
    {
        if (capacitorBank != nullptr && capacitorBank->isSprinting())
        {
            return capacitorBank->getMaximumOutputCurrent() *
                   can::capbank::CAPACITOR_BANK_OUTPUT_VOLTAGE;
        }

        return drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;
    }

    /**
     * Updates the desired wheel RPM based on the passed in x, y, and r components of
     * movement. See the class comment for x and y terminology (should be in right hand coordinate
     * system).
     *
     * @param[in] x The desired velocity of the wheels to move in the x direction.
     *      So if x=1000, the chassis algorithm will attempt to apply 1000 RPM to motors
     *      in order to move the chassis forward.
     * @param[in] y The desired velocity of the wheels to move in the y direction.
     *      See x param for further description.
     * @param[in] r The desired velocity of the wheels to rotate the chassis.
     *      See x param for further description.
     */
    virtual void setDesiredOutput(float x, float y, float r) = 0;

    /**
     * Zeros out the desired motor RPMs for all motors, but importantly doesn't zero out any other
     * chassis state information like desired rotation.
     */
    virtual void setZeroRPM() = 0;

    /**
     * Run chassis rotation PID on some actual turret angle offset.
     *
     * @param currentAngleError The error as an angle. For autorotation,
     * error between gimbal and center of chassis.
     * @param errD The derivative of currentAngleError.
     *
     * @retval a desired rotation speed (wheel speed)
     */
    mockable float chassisSpeedRotationPID(float currentAngleError, float errD);

    /**
     * When the desired rotational wheel speed is large, you can slow down your translational speed
     * to make a tighter and more controllable turn. This function that can be used to scale down
     * the translational chassis speed based on the desired rotational wheel speed.
     *
     * @param chassisRotationDesiredWheelspeed The desired rotational component of the chassis, in
     * wheel RPM.
     * @return A value between [0, 1] that is inversely proportional to the square of
     * chassisRotationDesiredWheelspeed. You then multiply your desired translational RPM by this
     * value.
     */
    mockable float calculateRotationTranslationalGain(float chassisRotationDesiredWheelspeed);

    /**
     * @return The actual chassis velocity in chassis relative frame, as a vector <vx, vy, vz>,
     *      where vz is rotational velocity. This is the velocity calculated from the chassis's
     *      encoders. Units: m/s
     */
    virtual modm::Matrix<float, 3, 1> getActualVelocityChassisRelative() const override = 0;

    const char* getName() const override { return "Chassis"; }

    mockable inline float getDesiredRotation() const { return desiredRotation; }

    static modm::Pair<int, float> lastComputedMaxWheelSpeed;
    static can::capbank::CapacitorBank* capacitorBank;

    float desiredRotation = 0;

    tap::communication::sensors::current::CurrentSensorInterface* currentSensor;

    CapBankPowerLimiter chassisPowerLimiter;

    virtual void limitChassisPower() = 0;

    /**
     * Converts the velocity matrix from raw RPM to wheel velocity in m/s.
     */
    inline modm::Matrix<float, 4, 1> convertRawRPM(const modm::Matrix<float, 4, 1>& mat) const
    {
        static constexpr float ratio = 2.0f * M_PI * CHASSIS_GEARBOX_RATIO / 60.0f;
        return mat * ratio;
    }

    virtual float mpsToRpm(float mps) const = 0;

};  // class HolonomicChassisSubsystem

}  // namespace chassis

}  // namespace aruwsrc

#endif  // HOLONOMIC_CHASSIS_SUBSYSTEM_HPP_
