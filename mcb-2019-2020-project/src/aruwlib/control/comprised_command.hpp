#ifndef __COMPRISED_COMMAND_HPP__
#define __COMPRISED_COMMAND_HPP__

#include "command.hpp"
#include <modm/container/dynamic_array.hpp>
#include <modm/container/smart_pointer.hpp>

namespace aruwlib
{

namespace control
{


class ComprisedCommand : public Command
{
 public:
    virtual void initialize() = 0;

    virtual void execute() = 0;

    virtual void end(bool interrupted) = 0;
};

}  // namespace control

}  // namespace aruwlib

#endif
