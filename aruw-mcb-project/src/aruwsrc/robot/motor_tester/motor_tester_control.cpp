#include "aruwsrc/robot/motor_tester/motor_subsystem.hpp"
#include "aruwsrc/robot/motor_tester/motor_command.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/control/command_mapping/joystick_analog_remote_mapping.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "tap/drivers.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/command_mapping/hold_command_mapping.hpp"


// Just using bare tap drivers because nothing else is needed
tap::Drivers drivers;
aruwsrc::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(&drivers);

tap::motor::DjiMotor m3508(&drivers, tap::motor::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "3508");
tap::motor::DjiMotor m2006(&drivers, tap::motor::MOTOR2, tap::can::CanBus::CAN_BUS1, false, "2006");
MotorSubsystem m3508Subsystem(drivers, m3508);
MotorSubsystem m2006Subsystem(drivers, m2006);


tap::control::JoystickAnalogRemoteMapping leftVerticalJoystick(drivers.remote, tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
MotorCommand m3508Command(m3508Subsystem, leftVerticalJoystick);


tap::control::HoldCommandMapping rightSwitchDown(&drivers, {&m3508Command}, tap::control::RemoteMapState(tap::communication::serial::Remote::Switch::LEFT_SWITCH, tap::communication::serial::Remote::SwitchState::DOWN));
// tap::control::HoldCommandMapping rightSwitchMid(tap::communication::serial::Remote::Switch::LEFT_SWITCH, tap::communication::serial::Remote::SwitchState::MID);


void initSubsystemCommands(tap::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(&remoteSafeDisconnectFunction);
    drivers->commandScheduler.registerSubsystem(&m2006Subsystem);
    drivers->commandScheduler.registerSubsystem(&m3508Subsystem);
    drivers->commandMapper.addMap(&rightSwitchDown);
    // drivers->commandMapper.addMap(rightSwitchMid);
}
