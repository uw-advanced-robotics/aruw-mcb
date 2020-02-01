#ifndef __HERO_SHOOT_COMPRISED_COMMAND_HPP__
#define __HERO_SHOOT_COMPRISED_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
#include "agitator_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "agitator_rotate_command.hpp"
#include "agitator_unjam_command.hpp"
#include "shoot_steady_comprised_command.hpp"

namespace aruwsrc
{

namespace agitator
{

class HeroShootComprisedCommand : public aruwlib::control::Command
{
public:
    HeroShootComprisedCommand(
        AgitatorSubsystem* waterWheel,
        AgitatorSubsystem* pusher,
        float agitatorChangeAngle,
        float pusherChangeAngle,
        float maxUnjamAngle,
        float agitatorDesiredRotateTime,
        float pusherDesiredRotateTime,
        float minAgitatorRotateTime,
        bool useSensorInput
    );

    void initialize(void);

    void execute(void);

    void end(bool interrupted);

    bool isFinished(void) const;
    
private:
    // static constexpr float AGITATOR_ANGLE_INCREMENT = aruwlib::algorithms::PI / 50.0f;

    AgitatorSubsystem* connectedAgitator1;
    AgitatorSubsystem* connectedAgitator2;

    ShootComprisedCommand wwRotateCommand;
    AgitatorRotateCommand pusherRotateCommand;

    bool useSensorInput;
};

}  // namespace control

}  // namespace aruwsrc

#endif
