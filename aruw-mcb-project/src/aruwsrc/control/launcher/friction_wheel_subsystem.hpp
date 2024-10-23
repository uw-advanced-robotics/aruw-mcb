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

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "modm/math/filter/pid.hpp"

#include "friction_wheel_test_command.hpp"
#include "launcher_constants.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::can
{
class TurretMCBCanComm;
}

namespace aruwsrc::control::launcher
{
/**
 * A subsystem which regulates the speed of a two wheel shooter system using velocity PID
 * controllers. Allows the user to specify the desired launch speed of the shooter.
 */
class FrictionWheelSubsystem : public tap::control::Subsystem
{
    friend class FrictionWheelTestCommand;

public:
    /**
     * Creates a new friction wheel subsystem
     */
    FrictionWheelSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorId leftMotorId,
        tap::motor::MotorId rightMotorId,
        tap::can::CanBus canBus,
        aruwsrc::can::TurretMCBCanComm *turretMCB);

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

    mockable float getDesiredFrictionWheelSpeed() const
    {
        return launchSpeedToFrictionWheelRpm(desiredLaunchSpeed);
    }

    /**
     * @return The average measured friction wheel speed of the launcher in RPM.
     */
    float getCurrentFrictionWheelSpeed() const;

    /**
     * Updates flywheel RPM ramp by elapsed time and sends motor output.
     */
    void refresh() override;

    void refreshSafeDisconnect() override
    {
        leftWheel.setDesiredOutput(0);
        rightWheel.setDesiredOutput(0);
    }

    const char *getName() const override { return "Friction wheels"; }

protected:
    /// The maximum launch speed that the user can request. The launch speed is limited between [0,
    /// MAX_DESIRED_LAUNCH_SPEED].
    static constexpr float MAX_DESIRED_LAUNCH_SPEED =
        LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT
            [MODM_ARRAY_SIZE(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT) - 1]
                .first;

    /// The maximum measured launch speed if the max desired launch speed is requested. This is a
    /// large overestimate on purpose--it is useful for providing an upper bound on the possible
    /// measured launch speed in case the measured launch speed is garbage.
    static constexpr float MAX_MEASURED_LAUNCH_SPEED = MAX_DESIRED_LAUNCH_SPEED + 10.0f;

    tap::Drivers *drivers;

private:
    modm::interpolation::Linear<modm::Pair<float, float>> launchSpeedLinearInterpolator;

    modm::Pid<float> velocityPidLeftWheel;

    modm::Pid<float> velocityPidRightWheel;

    float desiredLaunchSpeed;

    uint32_t prevTime = 0;

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

    aruwsrc::can::TurretMCBCanComm *turretMCB;

    FrictionWheelTestCommand frictionTestCommand;

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
