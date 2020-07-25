#ifndef XAXIS_COMMAND_HPP_
#define XAXIS_COMMAND_HPP_

#include "src/aruwlib/control/command.hpp"
#include "xaxis_subsystem.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{
namespace engineer
{
class XAxisCommand : public Command
{
 public:

    explicit XAxisCommand(XAxisSubsystem* subsystem, XAxisSubsystem::Position positionYAxis);

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;

 private:
    XAxisSubsystem* subsystemYAxis;
    float displacement; 
    XAxisSubsystem::Position positionYAxis; 
};  // class XAxisCommand
}  // namespace engineer
}  // namespace aruwsrc

#endif  // XAXIS_COMMAND_HPP_
