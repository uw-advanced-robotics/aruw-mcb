#ifndef __ENGINEER_WRIST_CALIBRATE_COMMAND_HPP__
#define __ENGINEER_WRIST_CALIBRATE_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
#include "engineer_wrist_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

class EngineerWristCalibrateCommand : public aruwlib::control::Command
{
public:
    EngineerWristCalibrateCommand(EngineerWristSubsystem* wrist);

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;

private:
    EngineerWristSubsystem* wrist;
};

}  // namespace control

}  // namespace aruwsrc

#endif  // __ENGINEER_WRIST_CALIBRATE_COMMAND_HPP__