#include "grab_box_comprised_command.hpp"

namespace aruwsrc
{

namespace engineer
{

GrabBoxComprisedCommand::GrabBoxComprisedCommand(
    GrabberSubsystem* grabber,
    WristSubsystem* wrist,
    float wristAngleChange,
    float wristRotateTime) :
    connectedGrabber(grabber),
    grabberCommand(grabber),
    connectedWrist(wrist),
    wristOutCommand(wrist, wristAngleChange, wristRotateTime),
    grabSequenceCommencing(false)
    {
        this->comprisedCommandScheduler.registerSubsystem(grabber);
        this->addSubsystemRequirement(dynamic_cast<Subsystem*>(grabber));

        this->comprisedCommandScheduler.registerSubsystem(wrist);
        this->addSubsystemRequirement(dynamic_cast<Subsystem*>(wrist));
    }

void GrabBoxComprisedCommand::initialize()
{
    // open grabber
    this->comprisedCommandScheduler.removeCommand(dynamic_cast<Command*>(&grabberCommand), true);

    // rotate wrist out
    this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&wristOutCommand));
}

void GrabBoxComprisedCommand::execute()
{
    if (wristOutCommand.isFinished()) {
        // close grabber
        this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&grabberCommand));
    }
    this->comprisedCommandScheduler.run();
}

void GrabBoxComprisedCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(
            dynamic_cast<Command*>(&grabberCommand), interrupted);
    this->comprisedCommandScheduler.removeCommand(
            dynamic_cast<Command*>(&wristOutCommand), interrupted);
}

bool GrabBoxComprisedCommand::isFinished() const
{
    // TODO, finished when the wrist is rotated and closed on box
    return ps == done;
}

}  // namspace engineer

}  // namespace aruwsrc