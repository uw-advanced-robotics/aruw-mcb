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

#ifndef __FRICTION_WHEEL_SUBSYSTEM_HPP__
#define __FRICTION_WHEEL_SUBSYSTEM_HPP__

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "tap/util_macros.hpp"

#include "modm/math/filter/pid.hpp"

namespace aruwsrc
{
namespace control
{
namespace launcher
{
/**
 * A subsystem which regulates the speed of a two wheel shooter system.
 */
class FrictionWheelSubsystem : public tap::control::Subsystem
{
public:
    static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR2;
    static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR1;
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    /**
     * Creates a new friction wheel subsystem with DJI motor1 and motor2
     * unless otherwise specified on CAN bus 1.
     */
    FrictionWheelSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorId leftMotorId = LEFT_MOTOR_ID,
        tap::motor::MotorId rightMotorId = RIGHT_MOTOR_ID)
        : tap::control::Subsystem(drivers),
          velocityPidLeftWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
          velocityPidRightWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
          desiredRpmRamp(0),
          leftWheel(drivers, leftMotorId, CAN_BUS_MOTORS, true, "left example motor"),
          rightWheel(drivers, rightMotorId, CAN_BUS_MOTORS, false, "right example motor")
    {
    }

    void initialize() override;

    /**
     * Sets target flywheel RPM.
     */
    mockable void setDesiredRpm(float desRpm);

    /**
     * Updates flywheel RPM ramp by elapsed time and sends motor output.
     */
    void refresh() override;

    void runHardwareTests() override;

    void onHardwareTestStart() override;

    void onHardwareTestComplete() override;

    const char *getName() override { return "Friction Wheel"; }

private:
    // speed of ramp when you set a new desired ramp speed [rpm / ms]
#ifdef TARGET_SENTINEL
    static constexpr float FRICTION_WHEEL_RAMP_SPEED = 0.5f;
#else
    static constexpr float FRICTION_WHEEL_RAMP_SPEED = 1.0f;
#endif

    static constexpr float PID_P = 20.0f;
    static constexpr float PID_I = 0.0f;
    static constexpr float PID_D = 0.0f;
    static constexpr float PID_MAX_ERROR_SUM = 0.0f;
    static constexpr float PID_MAX_OUTPUT = 16000.0f;

    modm::Pid<float> velocityPidLeftWheel;

    modm::Pid<float> velocityPidRightWheel;

    tap::algorithms::Ramp desiredRpmRamp;

    uint32_t prevTime = 0;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    tap::mock::DjiMotorMock leftWheel;
    tap::mock::DjiMotorMock rightWheel;

private:
#else
    tap::motor::DjiMotor leftWheel;
    tap::motor::DjiMotor rightWheel;
#endif
};

}  // namespace launcher

}  // namespace control

}  // namespace aruwsrc

#endif
