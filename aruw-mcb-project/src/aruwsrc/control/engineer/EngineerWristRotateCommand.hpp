#ifndef ENGINEER_WRIST_ROTATE_COMMAND_HPP_
#define ENGINEER_WRIST_ROTATE_COMMAND_HPP_

#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/algorithms/ramp.hpp>
#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/control/command.hpp>

namespace aruwsrc
{
namespace engineer
{
class EngineerWristSubsystem;
class EngineerWristRotateCommand : aruwlib::control::Command
{
public:
    static constexpr uint32_t WRIST_MIN_ROTATE_TIME = 300;

    EngineerWristRotateCommand(
        EngineerWristSubsystem* wrist,
        float wristAngle,
        float wristRotateTime = WRIST_MIN_ROTATE_TIME);

    const char* getName() const override { return "rotate engineer wrist"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

private:
    static constexpr float WRIST_ROTATE_COMMAND_PERIOD = 3;

    EngineerWristSubsystem* connectedWrist;

    float wristAngle;

    aruwlib::algorithms::Ramp wristRotateSetpoint;
    aruwlib::algorithms::Ramp wristRotateSetpointRight;

    float wristDesiredRotateTime;

    aruwlib::arch::MilliTimeout wristMinRotateTime;
};
}  // namespace engineer
}  // namespace aruwsrc

#endif  // __ENGINEER_WRIST_IN_COMMAND_HPP__