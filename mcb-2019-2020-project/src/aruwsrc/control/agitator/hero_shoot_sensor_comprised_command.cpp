#include "hero_shoot_sensor_comprised_command.hpp"
#include "agitator_rotate_command.hpp"
#include "agitator_unjam_command.hpp"
#include "src/aruwlib/control/command_scheduler.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace agitator
{

HeroShootSensorComprisedCommand::HeroShootSensorComprisedCommand(
    AgitatorSubsystem* waterWheel,
    AgitatorSubsystem* pusher,
    float agitatorChangeAngle,
    float pusherChangeAngle,
    float maxUnjamAngle,
    float agitatorDesiredRotateTime,
    float pusherRotateTime,
    float minAgitatorRotateTime) :
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
        pusherRotateTime,
        minAgitatorRotateTime
    ),
    unjamWWCommand(waterWheel, maxUnjamAngle)
{
    this->addSubsystemRequirement(dynamic_cast<Subsystem*>(waterWheel));
    this->addSubsystemRequirement(dynamic_cast<Subsystem*>(pusher));
}

void HeroShootSensorComprisedCommand::initialize()
{
    // if (clip is not full) {
        this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&wwRotateCommand));
    // }
    // if (ball is loaded) {
        this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&pusherRotateCommand));
    // }
}

void HeroShootSensorComprisedCommand::execute()
{
    this->comprisedCommandScheduler.run();
}

void HeroShootSensorComprisedCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(
        dynamic_cast<Command*>(&unjamWWCommand), interrupted);
    this->comprisedCommandScheduler.removeCommand(
        dynamic_cast<Command*>(&wwRotateCommand), interrupted);
    this->comprisedCommandScheduler.removeCommand(
        dynamic_cast<Command*>(&pusherRotateCommand), interrupted);
}

bool HeroShootSensorComprisedCommand::isFinished() const
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
