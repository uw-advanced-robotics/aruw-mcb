#ifndef __AGITATOR_SHOOT_COMPRISED_COMMANDS_HPP__
#define __AGITATOR_SHOOT_COMPRISED_COMMANDS_HPP__

#include "shoot_steady_comprised_command.hpp"

namespace aruwsrc
{

namespace agitator
{

class ShootFastComprisedCommand : public ShootComprisedCommand
{
 public:
    ShootFastComprisedCommand(AgitatorSubsystem* agitator17mm) :
    ShootComprisedCommand(
        agitator17mm,
        aruwlib::algorithms::PI / 5.0f,
        aruwlib::algorithms::PI / 2.0f,
        100.0f, 150.0f
    ) {}
};

class ShootSlowComprisedCommand : public ShootComprisedCommand
{
 public:
    ShootSlowComprisedCommand(AgitatorSubsystem* agitator17mm) :
    ShootComprisedCommand(
        agitator17mm,
        aruwlib::algorithms::PI / 5.0f,
        aruwlib::algorithms::PI / 2.0f,
        300.0f, 300.0f
    ) {}
};

}  // namespace agitator

}  // namespace aruwsrc

#endif
