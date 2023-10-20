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

#if defined(TARGET_TESTBED)

#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/agitator/constants/agitator_constants.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/robot_control.hpp"
#include "aruwsrc/robot/testbed/testbed_drivers.hpp"

#include "aruwsrc/control/agitator/manual_fire_rate_reselection_manager.hpp"
#include "aruwsrc/control/governor/fire_rate_limit_governor.hpp"
#include "tap/control/governor/governor_limited_command.hpp"


using namespace aruwsrc::agitator;
using namespace aruwsrc::control::agitator;
using namespace tap::control::setpoint;
using namespace aruwsrc::control::governor;
using namespace tap::control::governor;



using namespace aruwsrc::testbed;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace testbed_control
{

// Motor you just want to spin real fast
tap::motor::MotorId spinMotorID = tap::motor::MotorId::MOTOR2;

tap::motor::DjiMotor spinMotor(
    drivers(),
    spinMotorID,
    tap::can::CanBus::CAN_BUS1,
    false,
    "Testbed spinny motor");

// Agitator motor
VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG);

// An agitator??
MoveIntegralCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

ManualFireRateReselectionManager manualFireRateReselectionManager;
FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

GovernorLimitedCommand<1> rotateAndUnjamAgitatorButLimited(
    {&agitator},
    rotateAndUnjamAgitator,
    {&fireRateLimitGovernor});


void initializeSubsystems()
{
    spinMotor.initialize();
    agitator.initialize();

	manualFireRateReselectionManager.setFireRate(30);
}

void registerTestbedSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&agitator);
}

void setDefaultTestbedCommands(Drivers *drivers)
{
	agitator.setDefaultCommand(&rotateAndUnjamAgitatorButLimited);
}

void startTestbedCommands(Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&rotateAndUnjamAgitatorButLimited);

	spinMotor.setDesiredOutput(10'000);

}

}  // namespace testbed_control

namespace aruwsrc::testbed
{
void initSubsystemCommands(aruwsrc::testbed::Drivers *drivers)
{
    testbed_control::initializeSubsystems();
    testbed_control::registerTestbedSubsystems(drivers);
    testbed_control::setDefaultTestbedCommands(drivers);
    testbed_control::startTestbedCommands(drivers);
}
}  // namespace aruwsrc::testbed

#endif
