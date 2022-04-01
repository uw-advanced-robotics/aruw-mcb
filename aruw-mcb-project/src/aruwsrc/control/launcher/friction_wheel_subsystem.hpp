/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef FRICTION_WHEEL_SUBSYSTEM_HPP_
#define FRICTION_WHEEL_SUBSYSTEM_HPP_

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/constants.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "modm/math/filter/pid.hpp"

#include "launcher_constants.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::launcher
{
/**
 * A subsystem which regulates the speed of a two wheel shooter system using velocity PID
 * controllers. Allows the user to specify the desired launch speed of the shooter.
 */
class FrictionWheelSubsystem : public tap::control::Subsystem
{
public:
    /**
     * Creates a new friction wheel subsystem with DJI motor1 and motor2
     * unless otherwise specified on CAN bus 1.
     */
    FrictionWheelSubsystem(
        aruwsrc::Drivers *drivers,
        tap::motor::MotorId leftMotorId = LEFT_MOTOR_ID,
        tap::motor::MotorId rightMotorId = RIGHT_MOTOR_ID);

    void initialize() override;

    /**
     * Set the projectile launch speed - at what speed the pellets
     * will come out of the physical friction wheel launcher.
     * Speed is limited to a range of values defined by constants of
     * this subsystem.
     *
     * @param[in] speed The launch speed in m/s.
     */
    mockable void setDesiredLaunchSpeed(float speed);

    mockable float getDesiredLaunchSpeed() const { return desiredLaunchSpeed; }

    /**
     * Updates flywheel RPM ramp by elapsed time and sends motor output.
     */
    void refresh() override;

    void runHardwareTests() override;

    void onHardwareTestStart() override;

    void onHardwareTestComplete() override;

    const char *getName() override { return "Friction wheels"; }

protected:
    aruwsrc::Drivers *drivers;

private:
    aruwsrc::Drivers *drivers;

    modm::interpolation::Linear<modm::Pair<float, float>> launchSpeedLinearInterpolator;

    modm::Pid<float> velocityPidLeftWheel;

    modm::Pid<float> velocityPidRightWheel;

    float desiredLaunchSpeed;

    uint32_t prevTime = 0;

    float predictedLaunchSpeed = 0;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    tap::algorithms::Ramp desiredRpmRamp;

    testing::NiceMock<tap::mock::DjiMotorMock> leftWheel;
    testing::NiceMock<tap::mock::DjiMotorMock> rightWheel;

private:
#else
    tap::algorithms::Ramp desiredRpmRamp;

    tap::motor::DjiMotor leftWheel;
    tap::motor::DjiMotor rightWheel;
#endif

    /**
     * @param[in] launchSpeed Some launch speed in m/s. The speed will be
     *      capped between [0, LAUNCH_SPEED_MAX] m/s. If the speed is <= LAUNCH_SPEED_MIN_CUTOFF
     *      m/s, the speed will be rounded down to 0.
     * @return A friction wheel RPM that the given `launchSpeed` maps to.
     */
    float launchSpeedToFrictionWheelRpm(float launchSpeed) const;
};

}  // namespace aruwsrc::control::launcher

#endif  // FRICTION_WHEEL_SUBSYSTEM_HPP_
