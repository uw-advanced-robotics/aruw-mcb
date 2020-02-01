#include "hero_shoot_comprised_command.hpp"
#include "agitator_rotate_command.hpp"
#include "agitator_unjam_command.hpp"
#include "src/aruwlib/control/command_scheduler.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace agitator
{

HeroShootComprisedCommand::HeroShootComprisedCommand(
    AgitatorSubsystem* waterWheel,
    AgitatorSubsystem* pusher,
    float agitatorChangeAngle,
    float pusherChangeAngle,
    float maxUnjamAngle,
    float agitatorDesiredRotateTime,
    float pusherDesiredRotateTime,
    float minAgitatorRotateTime,
    bool useSensorInput) :
    connectedAgitator1(waterWheel),
    connectedAgitator2(pusher),
    wwRotateCommand(
        waterWheel,
        agitatorChangeAngle,
        agitatorDesiredRotateTime,
        minAgitatorRotateTime
    ),
    pusherRotateCommand(
        pusher,
        pusherChangeAngle,
        pusherDesiredRotateTime,
        minAgitatorRotateTime
    ),
    unjamWWCommand(waterWheel, maxUnjamAngle),
    useSensorInput(useSensorInput)
{
    this->addSubsystemRequirement(dynamic_cast<Subsystem*>(waterWheel));
    this->addSubsystemRequirement(dynamic_cast<Subsystem*>(pusher));
}

void HeroShootComprisedCommand::initialize()
{
    // if (clip is not full || !useSensorInput) {
        this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&wwRotateCommand));
    // }
    // if (ball is loaded || !useSensorInput) {
        this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&pusherRotateCommand));
    // }
}

void HeroShootComprisedCommand::execute()
{
    this->comprisedCommandScheduler.run();
}

void HeroShootComprisedCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(
        dynamic_cast<Command*>(&unjamWWCommand), interrupted);
    this->comprisedCommandScheduler.removeCommand(
        dynamic_cast<Command*>(&wwRotateCommand), interrupted);
    this->comprisedCommandScheduler.removeCommand(
        dynamic_cast<Command*>(&pusherRotateCommand), interrupted);
}

bool HeroShootComprisedCommand::isFinished() const
{
    /// \todo make an actual condition for finishing
    return ((wwRotateCommand.isFinished()
        && !unjamSequenceCommencing)
        || (unjamWWCommand.isFinished()
        && unjamSequenceCommencing))
        && pusherRotateCommand.isFinished();
}

}  // namespace control

}  // namespace aruwsrc
