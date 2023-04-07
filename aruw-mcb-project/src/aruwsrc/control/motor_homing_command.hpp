#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/homeable_subsystem_interface.hpp"

namespace aruwsrc::control
{
class MotorHomingCommand : public tap::control::Command
{
public:
    static constexpr int32_t HOMING_MOTOR_OUTPUT = SHRT_MAX / 2;

    enum class HomingState
    {
        INITIATE_MOVE_TOWARD_LOWER_BOUND,
        MOVING_TOWARD_LOWER_BOUND,
        INITIATE_MOVE_TOWARD_UPPER_BOUND,
        MOVING_TOWARD_UPPER_BOUND,
        HOMING_COMPLETE
    };

    MotorHomingCommand(
        aruwsrc::control::HomeableSubsystemInterface& subsystem,
        tap::Drivers& drivers)
        : subsystem(subsystem),
          drivers(drivers)
    {
        addSubsystemRequirement(&subsystem);
    };

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool interrupt) override;

private:
    aruwsrc::control::HomeableSubsystemInterface& subsystem;
    tap::Drivers& drivers;
    HomingState homingState;
};  // class MotorHomingCommand
}  // namespace aruwsrc::control