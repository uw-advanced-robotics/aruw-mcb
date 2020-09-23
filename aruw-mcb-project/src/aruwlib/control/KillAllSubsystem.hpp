#ifndef KILL_ALL_SUBSYSTEM_HPP_
#define KILL_ALL_SUBSYSTEM_HPP_

#include <aruwlib/control/subsystem.hpp>

namespace aruwlib
{
namespace control
{
class KillAllSubsystem : public aruwlib::control::Subsystem
{
public:
    explicit KillAllSubsystem(aruwlib::Drivers *drivers) : aruwlib::control::Subsystem(drivers) {}
};  // class KillAllSubsystem
}  // namespace control
}  // namespace aruwlib

#endif  // KILL_ALL_SUBSYSTEM_HPP_
