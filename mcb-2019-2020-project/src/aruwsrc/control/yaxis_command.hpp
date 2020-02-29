#ifndef __COMMAND_XAXIS_HPP__
#define __COMMAND_XAXIS_HPP__

#include "src/aruwlib/control/command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class YAxisSubsystem;

class YAxisCommand : public Command
{
 public:

    explicit YAxisCommand(YAxisSubsystem* subsystem = nullptr);

    void initialize(void);

    void execute(enum YAxisSubsystem::Position);

    void end(bool interrupted);

    bool isFinished(void) const;

    void interrupted(void);

 private:
    YAxisSubsystem* subsystemYAxis;
    float displacement; 
    YAxisSubsystem::Position positionYAxis; 
};

}

}

#endif