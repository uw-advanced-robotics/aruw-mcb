#ifndef __AGITATOR_CALIBRATE_COMMAND_HPP__
#define __AGITATOR_CALIBRATE_COMMAND_HPP__

#include <aruwlib/control/command.hpp>

#include "agitator_subsystem.hpp"

namespace aruwsrc
{
namespace agitator
{
/**
 * Default command that can be used to calibrate the agitator (i.e. spam calls
 * agitatorCalibrateHere). By default, the agitator will keep calling agitatorCalibrateHere
 * until the agitator is connected, however this command is for the following:
 *  - a placeholder command initially
 *  - allows you to recalibrate an agitator that has already been calibrated if necessary
 */
template <typename Drivers>
class AgitatorCalibrateCommand : public aruwlib::control::Command<Drivers>
{
public:
    explicit AgitatorCalibrateCommand(AgitatorSubsystem<Drivers>* agitator)
    {
        this->addSubsystemRequirement(
            dynamic_cast<aruwlib::control::Subsystem<Drivers>*>(agitator));
    }

    const char* getName() const override { return "agitator calibrate command"; }

    void initialize() override { agitator->agitatorCalibrateHere(); }

    void execute() override { agitator->agitatorCalibrateHere(); }

    void end(bool) override {}

    bool isFinished() const override { return agitator->isAgitatorCalibrated(); }

private:
    AgitatorSubsystem<Drivers>* agitator;
};

}  // namespace agitator

}  // namespace aruwsrc

#endif
