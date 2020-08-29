#ifndef __AGITATOR_SHOOT_COMPRISED_COMMANDS_HPP__
#define __AGITATOR_SHOOT_COMPRISED_COMMANDS_HPP__

#include "agitator_shoot_comprised_command.hpp"

namespace aruwsrc
{
namespace agitator
{
template <typename Drivers> class ShootFastComprisedCommand : public ShootComprisedCommand<Drivers>
{
public:
    explicit ShootFastComprisedCommand(AgitatorSubsystem<Drivers>* agitator17mm)
        : ShootComprisedCommand<Drivers>(
              agitator17mm,
              aruwlib::algorithms::PI / 5.0f,
              aruwlib::algorithms::PI / 2.0f,
              50,
              20)
    {
    }
};

template <typename Drivers> class ShootSlowComprisedCommand : public ShootComprisedCommand<Drivers>
{
public:
    explicit ShootSlowComprisedCommand(AgitatorSubsystem<Drivers>* agitator17mm)
        : ShootComprisedCommand<Drivers>(
              agitator17mm,
              aruwlib::algorithms::PI / 5.0f,
              aruwlib::algorithms::PI / 2.0f,
              300,
              100)
    {
    }
};
}  // namespace agitator

}  // namespace aruwsrc

#endif
