#ifndef __AGITATOR_CALIBRATE_COMMAND_HPP__
#define __AGITATOR_CALIBRATE_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
#include "agitator_subsystem.hpp"

namespace aruwsrc
{

namespace agitator
{

class AgitatorCalibrateCommand : public aruwlib::control::Command
{
 public:    
    AgitatorCalibrateCommand(AgitatorSubsystem* agitator);
    
    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;

 private:
    AgitatorSubsystem* agitator;
};

}  // namespace control

}  // namespace aruwsrc


#endif
