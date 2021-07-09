#ifndef YAXIS_COMMAND_HPP_
#define YAXIS_COMMAND_HPP_

#include "aruwlib/control/command.hpp"

#include "yaxis_subsystem.hpp"

namespace aruwsrc
{
namespace engineer
{
class YAxisCommand : public aruwlib::control::Command
{
public:
    YAxisCommand(YAxisSubsystem* subsystem, YAxisSubsystem::Position positionYAxis);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "y axis command"; }

private:
    static constexpr float COMPLETE_DIFF_CRITERIA = 0.1f;

    YAxisSubsystem* subsystemYAxis;
    float displacement;
    YAxisSubsystem::Position positionYAxis;
};  // class XAxisCommand
}  // namespace engineer
}  // namespace aruwsrc

#endif  // YAXIS_COMMAND_HPP_
