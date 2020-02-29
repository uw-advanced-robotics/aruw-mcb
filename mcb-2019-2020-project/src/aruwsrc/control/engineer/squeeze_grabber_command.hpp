/**
 * This code is part of aruw's repository
 * this is Engineer grabber mechanism's command code
 * 
 */

#ifndef __COMMAND_GRABBER_HPP__
#define __COMMAND_GRABBER_HPP__

#include "src/aruwlib/control/command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace engineer
{

class GrabberSubsystem;

class GrabberCommand : public Command
{
 public:
    explicit GrabberCommand(GrabberSubsystem* subsystem);

    void initialize(void);

    void execute(void);

    void end(bool interrupted);

    bool isFinished(void) const;

    void interrupted(void);

 private: 
    GrabberSubsystem* subsystemGrabber;
    
};

}  // namespace engineer

}  // namespace aruwsrc
#endif
