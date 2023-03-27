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

#include "velocity_agitator_subsystem.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

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
    tap::Drivers* drivers,
    const tap::algorithms::SmoothPidConfig& pidParams,
    const VelocityAgitatorSubsystemConfig& agitatorSubsystemConfig)
    : tap::control::Subsystem(drivers),
      config(agitatorSubsystemConfig),
      velocityPid(pidParams),
      jamChecker(this, config.jammingVelocityDifference, config.jammingTime),
      agitatorMotor(
          drivers,
          config.agitatorMotorId,
          config.agitatorCanBusId,
          config.isAgitatorInverted,
          "agitator motor")
{
    assert(config.jammingVelocityDifference >= 0);
}

void VelocityAgitatorSubsystem::initialize() { agitatorMotor.initialize(); }

void VelocityAgitatorSubsystem::refresh()
{
    if (!isOnline())
    {
        agitatorIsCalibrated = false;
    }

    if (!agitatorIsCalibrated)
    {
        if (!calibrateHere())
        {
            return;
        }
    }

    runVelocityPidControl();

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
    velocitySetpoint = 0.0f;
    clearJam();
    return true;
}

float VelocityAgitatorSubsystem::getCurrentValueIntegral() const
{
    if (!agitatorIsCalibrated)
    {
        return 0.0f;
    }
    return getUncalibratedAgitatorAngle() - agitatorCalibratedZeroAngle;
}

float VelocityAgitatorSubsystem::getUncalibratedAgitatorAngle() const
{
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

    const float velocityError = velocitySetpoint - getCurrentValue();

    velocityPid.runControllerDerivateError(velocityError, dt);

    agitatorMotor.setDesiredOutput(
        velocityPid.getOutput() + velocitySetpoint * config.velocityPIDFeedForwardGain);
}

void VelocityAgitatorSubsystem::setSetpoint(float velocity)
{
    if (agitatorMotor.isMotorOnline())
    {
        velocitySetpoint = velocity;
    }
}
}  // namespace aruwsrc::agitator
