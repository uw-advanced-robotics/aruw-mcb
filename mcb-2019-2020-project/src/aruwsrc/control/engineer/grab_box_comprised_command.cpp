#include "grab_box_comprised_command.hpp"

namespace aruwsrc
{

namespace engineer
{

GrabBoxComprisedCommand::GrabBoxComprisedCommand(
    WristSubsystem* wrist,
    GrabberSubsystem* grabber,
    float wristAngleChange,
    float wristRotateTime) :
    connectedGrabber(grabber),
    grabberCommand(grabber),
    connectedWrist(wrist),
    wristOutCommand(wrist, wristAngleChange, wristRotateTime),
    done(false)
    {
        this->comprisedCommandScheduler.registerSubsystem(grabber);
        this->addSubsystemRequirement(dynamic_cast<Subsystem*>(grabber));

        this->comprisedCommandScheduler.registerSubsystem(wrist);
        this->addSubsystemRequirement(dynamic_cast<Subsystem*>(wrist));
    }

void GrabBoxComprisedCommand::initialize()
{
    // grabber is open by default

    // rotate wrist out
    this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&wristOutCommand));
    
    done = false;
}

void GrabBoxComprisedCommand::execute()
{
    if (wristOutCommand.isFinished() && !done) {
        done = true;

        // close grabber
        this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&grabberCommand));

        // TODO:
        // Start timer to account for the grabber to close on a cube
        // Once the time is up activate the lift to the upwards position
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
    // TODO: finished when the wrist is rotated and the lift is up
    return done && connectedGrabber->getIsSqueezed();
}

}  // namspace engineer

}  // namespace aruwsrc