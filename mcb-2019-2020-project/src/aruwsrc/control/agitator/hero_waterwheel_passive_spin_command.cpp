#include "hero_waterwheel_passive_spin_command.hpp"
#include "shoot_steady_comprised_command.hpp"
#include "src/aruwlib/control/command_scheduler.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace agitator
{

HeroWaterWheelPassiveSpinCommand::HeroWaterWheelPassiveSpinCommand(
    AgitatorSubsystem* waterWheel,
    float agitatorChangeAngle,
    float maxUnjamAngle,
    float desiredRotateTime,
    float minAgitatorRotateTime) :
    connectedAgitator(waterWheel),
    rotateCommand(
        waterWheel,
        agitatorChangeAngle,
        maxUnjamAngle,
        desiredRotateTime,
        minAgitatorRotateTime
    )
{
    this->addSubsystemRequirement(dynamic_cast<Subsystem*>(waterWheel));
}

void HeroWaterWheelPassiveSpinCommand::initialize()
{
}

void HeroWaterWheelPassiveSpinCommand::execute()
{
    // if (clip is not full && !this->comprisedCommandScheduler->isCommandScheduled(rotateCommand)) {
        this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&rotateCommand));
    // }
    this->comprisedCommandScheduler.run();
}

void HeroWaterWheelPassiveSpinCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(
        dynamic_cast<Command*>(&rotateCommand), interrupted);
}

bool HeroWaterWheelPassiveSpinCommand::isFinished() const
{
    return false;
}

}  // namespace control

}  // namespace aruwsrc
