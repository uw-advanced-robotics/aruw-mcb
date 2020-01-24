#ifndef __SHOOT_COMPRISED_COMMAND_HPP__
#define __SHOOT_COMPRISED_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
#include "agitator_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "agitator_rotate_command.hpp"
#include "agitator_unjam_command.hpp"

namespace aruwsrc
{

namespace agitator
{

class ShootSteadyComprisedCommand : public aruwlib::control::Command
{
public:
    ShootSteadyComprisedCommand(
        AgitatorSubsystem* agitator,
        float agitatorChangeAngle,
        float agitatorRotateTime,
        float maxUnjamAngle
    );

    /**
     * The initial subroutine of a command.  Called once when the command is
     * initially scheduled.
     */
    void initialize(void);

    /**
     * The main body of a command.  Called repeatedly while the command is
     * scheduled.
     */
    void execute(void);

    /**
     * The action to take when the command ends.  Called when either the command
     * finishes normally, or when it interrupted/canceled.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    void end(bool interrupted);

    /**
     * Whether the command has finished.  Once a command finishes, the scheduler
     * will call its end() method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    bool isFinished(void) const;
    
private:
    // static constexpr float AGITATOR_ANGLE_INCREMENT = aruwlib::algorithms::PI / 50.0f;

    AgitatorSubsystem* connectedAgitator; 

    AgitatorRotateCommand agitatorRotateCommand;

    AgitatorUnjamCommand agitatorUnjamCommand;

    bool unjamSequenceCommencing;
};

}  // namespace control

}  // namespace aruwsrc

#endif
