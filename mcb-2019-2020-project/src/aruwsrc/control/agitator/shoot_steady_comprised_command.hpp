#ifndef __SHOOT_COMPRISED_COMMAND_HPP__
#define __SHOOT_COMPRISED_COMMAND_HPP__

#include "src/aruwlib/control/comprised_command.hpp"
#include "agitator_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "agitator_rotate_command.hpp"
#include "agitator_unjam_command.hpp"

namespace aruwsrc
{

namespace agitator
{

class ShootComprisedCommand : public aruwlib::control::ComprisedCommand
{
public:
    ShootComprisedCommand(
        AgitatorSubsystem* agitator,
        float agitatorChangeAngle,
        float maxUnjamAngle,
        float agitatorDesiredRotateTime,
        float minAgitatorRotateTime
    );

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;
    
private:
    AgitatorSubsystem* connectedAgitator; 

    AgitatorRotateCommand agitatorRotateCommand;

    AgitatorUnjamCommand agitatorUnjamCommand;

    bool unjamSequenceCommencing;
};

}  // namespace control

}  // namespace aruwsrc

#endif
