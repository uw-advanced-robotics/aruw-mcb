#ifndef __RESERVOIR_42MM_CALIBRATE_COMMAND__
#define __RESERVOIR_42MM_CALIBRATE_COMMAND__

#include "src/aruwlib/control/command.hpp"
#include "reservoir_42mm_subsystem.hpp"

namespace aruwsrc
{

namespace engineer
{

class Reservoir42mmCalibrateCommand : public aruwlib::control::Command
{
public:
    Reservoir42mmCalibrateCommand(Reservoir42mmSubsystem* reservoir);

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;

private:
    Reservoir42mmSubsystem* reservoir;
};

}  // namespace engineer

}  // namespace aruwsrc

#endif  // __RESERVOIR_42MM_CALIBRATE_COMMAND__