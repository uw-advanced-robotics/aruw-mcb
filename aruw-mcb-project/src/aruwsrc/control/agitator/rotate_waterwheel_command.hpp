#ifndef ROTATE_WATERWHEEL_COMMAND_HPP_
#define ROTATE_WATERWHEEL_COMMAND_HPP_

#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"

#include "agitator_subsystem.hpp"
#include "kicker_agitator_subsystem.hpp"

namespace aruwsrc::agitator
{
/**
 */
class RotateWaterwheelCommand : public tap::control::setpoint::MoveUnjamComprisedCommand
{
public:
    /** Angle the command tries to move the agitator whenever it is scheduled */
    static constexpr float WATERWHEEL_42MM_CHANGE_ANGLE = M_PI / 7;
    /** Max angle the agitator will move while unjamming */
    static constexpr float WATERWHEEL_42MM_MAX_UNJAM_ANGLE = M_PI / 35;
    /** Expected time for the water wheel to rotate the specified angle in ms */
    static constexpr uint32_t WATERWHEEL_42MM_ROTATE_TIME = 1000;
    /** How long the command should wait after reaching the target angle */
    static constexpr uint32_t WATERWHEEL_42MM_PAUSE_AFTER_ROTATE_TIME = 0;

    RotateWaterwheelCommand(
        aruwsrc::Drivers* drivers,
        AgitatorSubsystem* waterwheel,
        const KickerAgitatorSubsystem* kicker);

    bool isReady() override;

    bool isFinished() const override;

private:
    // Store instance of drivers to be able to access digital
    aruwsrc::Drivers* drivers;
    const KickerAgitatorSubsystem* kicker;
};  // class Waterwheel42mmLoadCommand

}  // namespace aruwsrc::agitator

#endif  // ROTATE_WATERWHEEL_COMMAND_HPP_
