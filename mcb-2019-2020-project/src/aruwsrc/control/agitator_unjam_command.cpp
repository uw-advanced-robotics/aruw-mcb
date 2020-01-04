#include "agitator_unjam_command.hpp"

namespace aruwsrc
{

namespace control
{

const float AgitatorUnjamCommand::agitatorSetpointToleranceDefault = aruwlib::algorithms::PI / 16;

const uint32_t AgitatorUnjamCommand::AGITATOR_MAX_WAIT_TIME = 130;

const float AgitatorUnjamCommand::MIN_AGITATOR_UNJAM_ANGLE = aruwlib::algorithms::PI / 4;

AgitatorUnjamCommand::AgitatorUnjamCommand(
    AgitatorSubsystem* agitator,
    float agitatorMaxUnjamAngle,
    uint32_t agitatorMaxWaitTime,
    float setpointTolerance)
    : currUnjamstate(AGITATOR_UNJAM_BACK),
    agitatorMaxWaitTime(agitatorMaxWaitTime),
    connectedAgitator(agitator),
    agitatorSetpointTolerance(setpointTolerance),
    agitatorUnjamAngleMax(agitatorMaxUnjamAngle),
    currAgitatorUnjamAngle(0),
    agitatorSetpointBeforeUnjam(0)
{
    // todo(matthew) make sure agitator max and min angles are reasonable
    this->addSubsystemRequirement(reinterpret_cast<aruwlib::control::Subsystem*>(agitator));
    agitatorUnjamRotateTimeout.stop();
}

void AgitatorUnjamCommand::initialize()
{
    // define a random time that the agitator will take to unjam backwards.
    agitatorUnjamRotateTimeout.restart(agitatorMaxWaitTime);

    // define a random unjam angle
    currAgitatorUnjamAngle = fmod(static_cast<double>(rand()),
        static_cast<double>(agitatorUnjamAngleMax - MIN_AGITATOR_UNJAM_ANGLE))
        + static_cast<double>(MIN_AGITATOR_UNJAM_ANGLE);

    // subtract this angle from the current angle
    currAgitatorUnjamAngle = connectedAgitator->getAgitatorEncoderToPosition() - currAgitatorUnjamAngle;

    // store the current setpoint angle to be referenced later
    agitatorSetpointBeforeUnjam = connectedAgitator->getAgitatorDesiredAngle();
    currUnjamstate = AGITATOR_UNJAM_BACK;
}

void AgitatorUnjamCommand::execute()
{
    switch (currUnjamstate)
    {
        case AGITATOR_UNJAM_BACK:
        {
            connectedAgitator->setAgitatorAngle(currAgitatorUnjamAngle);
            if (
                agitatorUnjamRotateTimeout.isExpired() ||
                fabs(static_cast<double>(connectedAgitator->getAgitatorEncoderToPosition() -
                connectedAgitator->getAgitatorDesiredAngle())) < static_cast<double>(agitatorSetpointTolerance)
            ) {
                // define a random time that the agitator will take to rotate forwards.
                agitatorUnjamRotateTimeout.restart(agitatorMaxWaitTime);

                // move on to unjam forward state
                currUnjamstate = AGITATOR_UNJAM_FORWARD;
            }
            break;
        }
        case AGITATOR_UNJAM_FORWARD:  // this is inherently different than just agitator_rotate_command
        {
            connectedAgitator->setAgitatorAngle(agitatorSetpointBeforeUnjam);
            // the agitator is still unjammed
            if (agitatorUnjamRotateTimeout.isExpired())
            {
                agitatorUnjamRotateTimeout.restart(agitatorMaxWaitTime);

                currAgitatorUnjamAngle = fmod(static_cast<double>(rand()),
                    static_cast<double>(agitatorUnjamAngleMax - MIN_AGITATOR_UNJAM_ANGLE))
                    + static_cast<double>(MIN_AGITATOR_UNJAM_ANGLE);

                currAgitatorUnjamAngle = agitatorSetpointBeforeUnjam - currAgitatorUnjamAngle;

                currUnjamstate = AGITATOR_UNJAM_BACK;
            }
            else if (
                fabs(static_cast<double>(connectedAgitator->getAgitatorEncoderToPosition() -
                connectedAgitator->getAgitatorDesiredAngle())) < static_cast<double>(agitatorSetpointTolerance)
            ) {
                currUnjamstate = FINISHED;
            }
            break;
        }
        case FINISHED:  // this could be only two states, but its simpler to understand and debug with three
        {
            break;
        }
    }
}

void AgitatorUnjamCommand::end(bool interrupted)
{
    if (interrupted) {}
    connectedAgitator->setAgitatorAngle(agitatorSetpointBeforeUnjam);
}

bool AgitatorUnjamCommand::isFinished(void) const
{
    return currUnjamstate == FINISHED;
}

}  // namespace control

}  // namespace aruwsrc
