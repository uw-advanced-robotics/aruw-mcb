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
    wristInCommand(wrist, wristAngleChange, wristRotateTime),
    wristOutCommand(wrist, -1.0f * wristAngleChange, wristRotateTime)
    {
        this->comprisedCommandScheduler.registerSubsystem(grabber);
        this->addSubsystemRequirement(dynamic_cast<Subsystem*>(grabber));

        this->comprisedCommandScheduler.registerSubsystem(wrist);
        this->addSubsystemRequirement(dynamic_cast<Subsystem*>(wrist));
    }

void GrabBoxComprisedCommand::initialize()
{
    // start sequence
    ps = out;

    // open grabber
    this->comprisedCommandScheduler.removeCommand(dynamic_cast<Command*>(&grabberCommand), true);

    // rotate wrist out
    this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&wristOutCommand));
}

void GrabBoxComprisedCommand::execute()
{
    switch (ps)
    {
        case out :
            // check if the out sequence is finished
            if (wristOutCommand.isFinished()) {
                ps = in;

                // close grabber
                this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&grabberCommand));

                // rotate wrist in
                this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&wristInCommand));
            }
            break;
        case in :
            // check if the in sequence is finished
            if (wristInCommand.isFinished()) {
                ps = done;
            }
            break;
    }
    this->comprisedCommandScheduler.run();
}

void GrabBoxComprisedCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(
            dynamic_cast<Command*>(&grabberCommand), interrupted);
    this->comprisedCommandScheduler.removeCommand(
            dynamic_cast<Command*>(&wristInCommand), interrupted);
    this->comprisedCommandScheduler.removeCommand(
            dynamic_cast<Command*>(&wristOutCommand), interrupted);
}

bool GrabBoxComprisedCommand::isFinished() const
{
    return ps == done;
}

}  // namspace engineer

}  // namespace aruwsrc