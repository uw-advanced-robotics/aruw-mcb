#ifndef ENGINEER_WRIST_CALIBRATE_COMMAND_HPP_
#define ENGINEER_WRIST_CALIBRATE_COMMAND_HPP_

#include <aruwlib/control/command.hpp>

namespace aruwsrc
{
namespace engineer
{
class EngineerWristSubsystem;
class EngineerWristCalibrateCommand : public aruwlib::control::Command
{
public:
    EngineerWristCalibrateCommand(EngineerWristSubsystem* wrist);

    const char* getName() const { return "Engineer Wrist Calibrate"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

private:
    EngineerWristSubsystem* wrist;
};
}  // namespace engineer
}  // namespace aruwsrc

#endif  // ENGINEER_WRIST_CALIBRATE_COMMAND_HPP_
