#ifndef __COMMAND_XAXIS_HPP__
#define __COMMAND_XAXIS_HPP__

#include "src/aruwlib/control/command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class XAxisSubsystem;

class XAxisCommand : public Command
{
 public:
    explicit XAxisCommand(XAxisSubsystem* subsystem = nullptr);

    void initialize(void);

    void execute(void);

    void end(bool interrupted);

    bool isFinished(void) const;

    void interrupted(void);

 private:
    float desiredPosition;
    float currentPosition;
    XAxisSubsystem* subsystemXAxis;
};

}

}

#endif