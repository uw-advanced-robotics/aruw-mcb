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

#include "aruwlib/algorithms/ramp.hpp"
#include "aruwlib/control/subsystem.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwlib/mock/dji_motor_mock.hpp"
#else
#include "aruwlib/motor/dji_motor.hpp"
#endif

#include "aruwlib/util_macros.hpp"

#include "modm/math/filter/pid.hpp"

namespace aruwsrc
{
namespace launcher
{
/**
 * A subsystem which regulates the speed of a two wheel shooter system.
 */
class FrictionWheelSubsystem : public aruwlib::control::Subsystem
{
public:
    /**
     * @brief Creates a new friction wheel subsystem with DJI motor1 and motor2
     * unless otherwise specified on CAN bus 1.
     *
     * @param[in] drivers Pointer to a drivers singleton object.
     * @param[in] pidP Proportioanl parameter for RPM PID controller.
     * @param[in] pidI Integral parameter for RPM PID controller.
     * @param[in] pidD Derivative parameter for RPM PID controller.
     * @param[in] pidMaxErrorSum Max integral sum for RPM PID controller.
     * @param[in] pidMaxOutput Max output for RPM PID controller.
     * @param[in] frictionWheelRampSpeed Speed of ramp when you set a new desired ramp speed
     *      [rpm / ms].
     * @param[in] leftMotorId DJI motor ID for left motor.
     * @param[in] rightMotorId DJI motor ID for right motor.
     * @param[in] canBus CAN bus that the friction wheels are connected to.
     */
    FrictionWheelSubsystem(
        aruwlib::Drivers *drivers,
        float pidP,
        float pidI,
        float pidD,
        float pidMaxErrorSum,
        float pidMaxOutput,
        float frictionWheelRampSpeed,
        aruwlib::motor::MotorId leftMotorId,
        aruwlib::motor::MotorId rightMotorId,
        aruwlib::can::CanBus canBus);

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
    const float FRICTION_WHEEL_RAMP_SPEED;

    modm::Pid<float> velocityPidLeftWheel;

    modm::Pid<float> velocityPidRightWheel;

    aruwlib::algorithms::Ramp desiredRpmRamp;

    uint32_t prevTime = 0;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    aruwlib::mock::DjiMotorMock leftWheel;
    aruwlib::mock::DjiMotorMock rightWheel;

private:
#else
    aruwlib::motor::DjiMotor leftWheel;
    aruwlib::motor::DjiMotor rightWheel;
#endif
};

}  // namespace launcher

}  // namespace aruwsrc

#endif
