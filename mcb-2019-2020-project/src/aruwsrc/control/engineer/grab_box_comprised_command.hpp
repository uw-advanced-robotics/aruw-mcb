#ifndef __GRAB_BOX_COMPRISED_COMMAND_HPP__
#define __GRAB_BOX_COMPRISED_COMMAND_HPP__

#include "src/aruwlib/control/comprised_command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace engineer
{

class GrabBoxComprisedCommand : public ComprisedCommand
{
 public:
    GrabBoxComprisedCommand();

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;

 private:
    // TODO
    // instantiate each subsystem, and commands needed
};

}  // namespace engineer

}  // namespace aruwsrc

#endif  // __GRAB_BOX_COMPRISED_COMMAND_HPP__