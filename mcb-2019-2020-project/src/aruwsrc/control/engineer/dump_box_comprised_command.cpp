#include "dump_box_comprised_command.hpp"

namespace aruwsrc
{

namespace engineer
{

DumpBoxComprisedCommand::DumpBoxComprisedCommand(
    GrabberSubsystem* grabber,
    WristSubsystem* wrist,
    float wristAngleChange,
    float wristRotateTime) :
    connectedGrabber(grabber),
    grabberCommand(grabber),
    connectedWrist(wrist),
    wristOutCommand(wrist, wristAngleChange, wristRotateTime),
    dumpSequenceCommencing(false)
    {
        this->comprisedCommandScheduler.registerSubsystem(grabber);
        this->addSubsystemRequirement(dynamic_cast<Subsystem*>(grabber));

        this->comprisedCommandScheduler.registerSubsystem(wrist);
        this->addSubsystemRequirement(dynamic_cast<Subsystem*>(wrist));
    };

void DumpBoxComprisedCommand::initialize()
{
    // close grabber

    // move "x" axis back
}

void DumpBoxComprisedCommand::execute()
{
    // once x axis is done moving back
    // rotate wrist to in position
}

void DumpBoxComprisedCommand::end(bool interrupted)
{
    // remove commands
}

bool DumpBoxComprisedCommand::isFinished() const
{
    // finished when the wrist rotation is done
}

}  // namespace engineer

}  // namespace aruwsrc