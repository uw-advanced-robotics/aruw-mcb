#ifndef __HERO_WW_SPIN_COMMAND_HPP__
#define __HERO_WW_SPIN_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
#include "agitator_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "shoot_steady_comprised_command.hpp"

namespace aruwsrc
{

namespace agitator
{

class HeroWaterWheelPassiveSpinCommand : public aruwlib::control::Command
{
public:
    HeroWaterWheelPassiveSpinCommand(
        AgitatorSubsystem* waterWheel,
        float agitatorChangeAngle,
        float maxUnjamAngle,
        float desiredRotateTime,
        float minAgitatorRotateTime
    );

    void initialize(void);

    void execute(void);

    void end(bool interrupted);

    bool isFinished(void) const;
    
private:
    // static constexpr float AGITATOR_ANGLE_INCREMENT = aruwlib::algorithms::PI / 50.0f;

    AgitatorSubsystem* connectedAgitator;

    ShootComprisedCommand rotateCommand;

    bool unjamSequenceCommencing;
};

}  // namespace control

}  // namespace aruwsrc

#endif
