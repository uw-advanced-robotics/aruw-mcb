#ifndef __HERO_SENSOR_SHOOT_COMPRISED_COMMAND_HPP__
#define __HERO_SENSOR_SHOOT_COMPRISED_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
#include "agitator_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "agitator_rotate_command.hpp"
#include "agitator_unjam_command.hpp"

namespace aruwsrc
{

namespace agitator
{

class HeroShootSensorComprisedCommand : public aruwlib::control::Command
{
public:
    HeroShootSensorComprisedCommand(
        AgitatorSubsystem* waterWheel,
        AgitatorSubsystem* pusher,
        float agitatorChangeAngle,
        float pusherChangeAngle,
        float maxUnjamAngle,
        float agitatorDesiredRotateTime,
        float pusherRotateTime,
        float minAgitatorRotateTime
    );

    void initialize(void);

    void execute(void);

    void end(bool interrupted);

    bool isFinished(void) const;
    
private:
    // static constexpr float AGITATOR_ANGLE_INCREMENT = aruwlib::algorithms::PI / 50.0f;

    AgitatorSubsystem* connectedAgitator1;
    AgitatorSubsystem* connectedAgitator2;

    AgitatorRotateCommand wwRotateCommand;
    AgitatorRotateCommand pusherRotateCommand;

    AgitatorUnjamCommand unjamWWCommand;

    bool unjamSequenceCommencing;
};

}  // namespace control

}  // namespace aruwsrc

#endif
