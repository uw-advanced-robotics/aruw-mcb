#ifndef __WRIST_CALIBRATE_COMMAND_HPP__
#define __WRIST_CALIBRATE_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
#include "wrist_subsystem.hpp"

namespace aruwsrc
{

namespace engineer
{

class WristCalibrateCommand : public aruwlib::control::Command
{
public:
    WristCalibrateCommand(WristSubsystem* wrist);

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;

private:
    WristSubsystem* wrist;
};

}  // namespace engineer

}  // namespace aruwsrc

#endif  // __WRIST_CALIBRATE_COMMAND_HPP__