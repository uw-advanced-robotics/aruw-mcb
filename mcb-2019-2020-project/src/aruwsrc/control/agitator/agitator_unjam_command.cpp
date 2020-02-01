#include "agitator_unjam_command.hpp"

namespace aruwsrc
{

namespace agitator
{

const uint32_t AgitatorUnjamCommand::AGITATOR_MAX_WAIT_TIME = 130;

AgitatorUnjamCommand::AgitatorUnjamCommand(
    AgitatorSubsystem* agitator,
    float agitatorMaxUnjamAngle,
    uint32_t agitatorMaxWaitTime) :
    currUnjamstate(AGITATOR_UNJAM_BACK),
    agitatorMaxWaitTime(agitatorMaxWaitTime),
    connectedAgitator(agitator),
    agitatorUnjamAngleMax(agitatorMaxUnjamAngle),
    currAgitatorUnjamAngle(0),
    agitatorSetpointBeforeUnjam(0)
{
    this->addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(agitator));
    agitatorUnjamRotateTimeout.stop();
}

void AgitatorUnjamCommand::initialize()
{
    // define a random time that the agitator will take to unjam backwards.
    agitatorUnjamRotateTimeout.restart(agitatorMaxWaitTime);

    // define a random unjam angle between [MIN_AGITATOR_UNJAM_ANGLE, agitatorUnjamAngleMax]
    currAgitatorUnjamAngle = fmod(static_cast<double>(rand()),
        static_cast<double>(agitatorUnjamAngleMax - MIN_AGITATOR_UNJAM_ANGLE))
        + static_cast<double>(MIN_AGITATOR_UNJAM_ANGLE);

    // subtract this angle from the current angle to rotate agitator backwards
    currAgitatorUnjamAngle = connectedAgitator->getAgitatorAngle() - currAgitatorUnjamAngle;

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
                fabs(static_cast<double>(connectedAgitator->getAgitatorAngle() -
                connectedAgitator->getAgitatorDesiredAngle())) < static_cast<double>(AGITATOR_SETPOINT_TOLERANCE)
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
            // reset the angle to what it was before unjamming
            connectedAgitator->setAgitatorAngle(agitatorSetpointBeforeUnjam);
            // the agitator is still unjammed
            if (agitatorUnjamRotateTimeout.isExpired())
            {
                // restart the timeout
                agitatorUnjamRotateTimeout.restart(agitatorMaxWaitTime);

                // define a new random angle, which will be used in the unjam back state
                currAgitatorUnjamAngle = fmod(static_cast<double>(rand()),
                    static_cast<double>(agitatorUnjamAngleMax - MIN_AGITATOR_UNJAM_ANGLE))
                    + static_cast<double>(MIN_AGITATOR_UNJAM_ANGLE);

                currAgitatorUnjamAngle = agitatorSetpointBeforeUnjam - currAgitatorUnjamAngle;

                currUnjamstate = AGITATOR_UNJAM_BACK;
            }
            else if (
                fabs(static_cast<double>(connectedAgitator->getAgitatorAngle() -
                connectedAgitator->getAgitatorDesiredAngle())) < static_cast<double>(AGITATOR_SETPOINT_TOLERANCE)
            ) {
                currUnjamstate = FINISHED;
            }
            break;
        }
        case FINISHED:  // this could be only two states, but its simpler to debug with three
        {
            break;
        }
    }
}

void AgitatorUnjamCommand::end(bool interrupted)
{
    if (interrupted) {
        connectedAgitator->setAgitatorAngle(agitatorSetpointBeforeUnjam);
    }
    connectedAgitator->setAgitatorAngle(agitatorSetpointBeforeUnjam);
}

bool AgitatorUnjamCommand::isFinished(void) const
{
    return currUnjamstate == FINISHED;
}

}  // namespace control

}  // namespace aruwsrc
