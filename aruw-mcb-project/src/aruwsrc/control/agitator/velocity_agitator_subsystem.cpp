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

#include "velocity_agitator_subsystem.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/errors/create_errors.hpp"

#include "aruwsrc/drivers.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "modm/math/filter/pid.hpp"

using namespace tap::motor;

namespace aruwsrc::agitator
{
VelocityAgitatorSubsystem::VelocityAgitatorSubsystem(
    aruwsrc::Drivers* drivers,
    const tap::algorithms::SmoothPidConfig& pidParams,
    const AgitatorSubsystemConfig& agitatorSubsystemConfig)
    : tap::control::Subsystem(drivers),
      config(agitatorSubsystemConfig),
      velocityPid(pidParams),
      jamChecker(this, config.jammingDistance, config.jammingTime),
      agitatorMotor(
          drivers,
          config.agitatorMotorId,
          config.agitatorCanBusId,
          config.isAgitatorInverted,
          "agitator motor")
{
    assert(config.jammingDistance >= 0);
}

void VelocityAgitatorSubsystem::initialize() { agitatorMotor.initialize(); }

void VelocityAgitatorSubsystem::refresh()
{
    if (!agitatorIsCalibrated)
    {
        if (!calibrateHere())
        {
            return;
        }
    }

    if (movementEnabled)
    {
        runVelocityPidControl();
    }

    if (jamChecker.check())
    {
        subsystemJamStatus = true;
    }
}

bool VelocityAgitatorSubsystem::calibrateHere()
{
    if (!isOnline())
    {
        return false;
    }
    agitatorCalibratedZeroAngle = getUncalibratedAgitatorAngle();
    agitatorIsCalibrated = true;
    clearJam();
    return true;
}

float VelocityAgitatorSubsystem::getPosition() const
{
    if (!agitatorIsCalibrated)
    {
        return 0.0f;
    }
    return getUncalibratedAgitatorAngle() - agitatorCalibratedZeroAngle;
}

float VelocityAgitatorSubsystem::getJamSetpointTolerance() const
{
    return jamChecker.getJamSetpointTolerance();
}

float VelocityAgitatorSubsystem::getUncalibratedAgitatorAngle() const
{
    // position is equal to the following equation:
    // position = 2 * PI / encoder resolution * unwrapped encoder value / gear ratio
    return (2.0f * M_PI / static_cast<float>(DjiMotor::ENC_RESOLUTION)) *
           agitatorMotor.getEncoderUnwrapped() / config.gearRatio;
}

void VelocityAgitatorSubsystem::runHardwareTests() {}

void VelocityAgitatorSubsystem::onHardwareTestStart() {}

void VelocityAgitatorSubsystem::runVelocityPidControl()
{
    const uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
    const uint32_t dt = curTime - prevTime;
    prevTime = curTime;

    const float velocityError = desiredVelocity - getVelocity();

    velocityPid.runControllerDerivateError(velocityError, dt);

    agitatorMotor.setDesiredOutput(velocityPid.getOutput());
}

void VelocityAgitatorSubsystem::setVelocitySetpoint(float velocity)
{
    if (agitatorMotor.isMotorOnline())
    {
        desiredVelocity = velocity;
    }
}
}  // namespace aruwsrc::agitator
