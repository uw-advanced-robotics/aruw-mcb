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

    void initialize(void);

    void execute(void);

    void end(bool interrupted);

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
