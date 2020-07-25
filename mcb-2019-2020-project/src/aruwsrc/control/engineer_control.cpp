#include <aruwlib/Drivers.hpp>
#include <aruwlib/communication/gpio/digital.hpp>
#include <aruwlib/control/command_scheduler.hpp>

#include "aruwsrc/control/engineer/extend_xaxis_command.hpp"
#include "aruwsrc/control/engineer/grabber_subsystem.hpp"
#include "aruwsrc/control/engineer/squeeze_grabber_command.hpp"
#include "aruwsrc/control/engineer/xaxis_subsystem.hpp"
#include "aruwsrc/control/engineer/yaxis_command.hpp"
#include "aruwsrc/control/engineer/yaxis_subsystem.hpp"

#if defined(TARGET_ENGINEER)

using namespace aruwsrc::engineer;
using namespace aruwlib::gpio;
using aruwlib::Drivers;
using aruwlib::control::CommandMapper;
using aruwlib::Remote;

namespace aruwsrc
{
namespace control
{
/* Define pins --------------------------------------------------------------*/
const Digital::OutputPin grabberPin = Digital::OutputPin::E;
const Digital::OutputPin xAxisPin = Digital::OutputPin::F;
const Digital::InputPin yAxisPin = Digital::InputPin::A;

/* define subsystems --------------------------------------------------------*/
GrabberSubsystem grabber(grabberPin);
XAxisSubsystem xAxis(xAxisPin);
YAxisSubsystem yAxis(yAxisPin);

/* define commands ----------------------------------------------------------*/
// For testing
YAxisCommand c1(&yAxis, YAxisSubsystem::Position::CENTER_DISTANCE);
YAxisCommand c2(&yAxis, YAxisSubsystem::Position::MAX_DISTANCE);
YAxisCommand c3(&yAxis, YAxisSubsystem::Position::MIN_DISTANCE);

/* register subsystems here -------------------------------------------------*/
void registerEngineerSubsystems()
{
    Drivers::commandScheduler.registerSubsystem(&grabber);
    Drivers::commandScheduler.registerSubsystem(&xAxis);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultEngineerCommands() {}

/* add any starting commands to the scheduler here --------------------------*/
void startEngineerCommands() {}

/* register io mappings here ------------------------------------------------*/
void registerEngineerIoMappings() {
    Drivers::commandMapper.addHoldRepeatMapping(
        CommandMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID),
        &c1);
    Drivers::commandMapper.addHoldRepeatMapping(
        CommandMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP),
        &c2);
    Drivers::commandMapper.addHoldRepeatMapping(
        CommandMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN),
        &c3);
}

void initSubsystemCommands()
{
    registerEngineerSubsystems();
    setDefaultEngineerCommands();
    startEngineerCommands();
    registerEngineerIoMappings();
}

}  // namespace control

}  // namespace aruwsrc

#endif
