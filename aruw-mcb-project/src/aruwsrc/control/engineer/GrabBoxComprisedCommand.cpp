#include "GrabBoxComprisedCommand.hpp"

#include "EngineerWristSubsystem.hpp"
#include "grabber_subsystem.hpp"

namespace aruwsrc
{
namespace engineer
{
GrabBoxComprisedCommand::GrabBoxComprisedCommand(
    aruwlib::Drivers* drivers,
    EngineerWristSubsystem* wrist,
    GrabberSubsystem* grabber)
    : aruwlib::control::ComprisedCommand(drivers),
      cmdState(GrabberState::EXTENDING_GRABBER),
      grabber(grabber),
      wrist(wrist),
      rotateWristOut(wrist, GRAB_BIN_ANGLE),
      rotateWristIn(wrist, DUMP_BIN_ANGLE),
      throwBin(wrist, GRAB_BIN_ANGLE, THROW_BIN_ROTATE_TIME)
{
    comprisedCommandScheduler.registerSubsystem(grabber);
    comprisedCommandScheduler.registerSubsystem(wrist);

    addSubsystemRequirement(grabber);
    addSubsystemRequirement(wrist);
}

void GrabBoxComprisedCommand::initialize()
{
    comprisedCommandScheduler.addCommand(&rotateWristOut);
    cmdState = GrabberState::EXTENDING_GRABBER;
}

void GrabBoxComprisedCommand::execute()
{
    switch (cmdState)
    {
        case GrabberState::EXTENDING_GRABBER:
        {
            if (rotateWristOut.isFinished())
            {
                grabber->setSqueezed(true);
                comprisedCommandScheduler.addCommand(&rotateWristIn);
                cmdState = GrabberState::RETREIVING;
                modm_fallthrough;
            }
            else
            {
                break;
            }
        }
        case GrabberState::RETREIVING:
        {
            if (rotateWristIn.isFinished())
            {
                dumpingTimer.restart(DUMP_TIME);
                cmdState = GrabberState::DUMPING;
                modm_fallthrough;
            }
            else
            {
                break;
            }
        }
        case GrabberState::DUMPING:
        {
            if (dumpingTimer.execute())
            {
                comprisedCommandScheduler.addCommand(&throwBin);
                cmdState = GrabberState::THROWING;
            }
            break;
        }
        case GrabberState::THROWING:
        {
            if (wrist->getWristAngle() >= THROW_BIN_ANGLE)
            {
                grabber->setSqueezed(false);
            }

            if (throwBin.isFinished())
            {
                cmdState = GrabberState::FINISHED;
            }
            break;
        }
        case GrabberState::FINISHED:
        {
            break;
        }
    }

    comprisedCommandScheduler.run();
}

void GrabBoxComprisedCommand::end(bool)
{
    grabber->setSqueezed(false);
    wrist->setWristAngle(DUMP_BIN_ANGLE);
}

bool GrabBoxComprisedCommand::isFinished() const { return cmdState == GrabberState::FINISHED; }
}  // namespace engineer
}  // namespace aruwsrc
