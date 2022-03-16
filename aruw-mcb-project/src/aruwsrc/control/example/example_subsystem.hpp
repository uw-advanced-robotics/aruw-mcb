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

/**
 * This is part of aruw's library.
 *
 * This is example code for running friction wheels. As you can
 * see, there is a generic update pid loop that is independent of
 * what command is given to the subsystem. Additionally, the
 * subsystem contains variables specific to the subsystem
 * (pid controllers, motors, etc). If a control loop is specific
 * to a command, it should NOT be in a subsystem. For example,
 * control code to pulse the friction wheels should be located
 * outside of this class because pulsing is a specific command.
 */

#ifndef EXAMPLE_SUBSYSTEM_HPP_
#define EXAMPLE_SUBSYSTEM_HPP_

#include "tap/control/command_scheduler.hpp"
#include "tap/control/subsystem.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "modm/math/filter/pid.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace control
{
class ExampleSubsystem : public tap::control::Subsystem
{
public:
    ExampleSubsystem(
        aruwsrc::Drivers* drivers,
        tap::motor::MotorId leftMotorId = LEFT_MOTOR_ID,
        tap::motor::MotorId rightMotorId = RIGHT_MOTOR_ID);

    void initialize() override;

    void setDesiredRpm(float desRpm);

    void refresh() override;

    void runHardwareTests() override;

    const char* getName() override { return "Example"; }

private:
    static const tap::motor::MotorId LEFT_MOTOR_ID;
    static const tap::motor::MotorId RIGHT_MOTOR_ID;
    const tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    const float PID_P = 5.0f;
    const float PID_I = 0.0f;
    const float PID_D = 1.0f;
    const float PID_MAX_ERROR_SUM = 0.0f;
    const float PID_MAX_OUTPUT = 16000;

    modm::Pid<float> velocityPidLeftWheel;

    modm::Pid<float> velocityPidRightWheel;

    float desiredRpm;

    void updateMotorRpmPid(
        modm::Pid<float>* pid,
        tap::motor::DjiMotor* const motor,
        float desiredRpm);

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

}  // namespace control

}  // namespace aruwsrc

#endif  // EXAMPLE_SUBSYSTEM_HPP_
