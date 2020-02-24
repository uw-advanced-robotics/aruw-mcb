#include "agitator_unjam_command.hpp"

namespace aruwsrc
{

namespace agitator
{

AgitatorUnjamCommand::AgitatorUnjamCommand(
    AgitatorSubsystem* agitator,
    float agitatorMaxUnjamAngle,
    uint32_t agitatorMaxWaitTime) :
    currUnjamstate(AGITATOR_UNJAM_BACK),
    agitatorMaxWaitTime(agitatorMaxWaitTime),
    connectedAgitator(agitator),
    agitatorUnjamAngleMax(agitatorMaxUnjamAngle),
    currAgitatorUnjamAngle(0.0f),
    agitatorSetpointBeforeUnjam(0.0f)
{
    if (agitatorMaxUnjamAngle < MIN_AGITATOR_UNJAM_ANGLE)
    {
        agitatorUnjamAngleMax = MIN_AGITATOR_UNJAM_ANGLE;
    }
    this->addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(agitator));
    agitatorUnjamRotateTimeout.stop();
}

void AgitatorUnjamCommand::initialize()
{
    // define a random time that the agitator will take to unjam backwards.
    agitatorUnjamRotateTimeout.restart(agitatorMaxWaitTime);

    // define a random unjam angle between [MIN_AGITATOR_UNJAM_ANGLE, agitatorUnjamAngleMax]

    // NOLINTNEXTLINE
    float randomUnjamAngle = fmodf(rand(), agitatorUnjamAngleMax - MIN_AGITATOR_UNJAM_ANGLE)
            + MIN_AGITATOR_UNJAM_ANGLE;

    // subtract this angle from the current angle to rotate agitator backwards
    currAgitatorUnjamAngle = connectedAgitator->getAgitatorAngle() - randomUnjamAngle;

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
            connectedAgitator->setAgitatorDesiredAngle(currAgitatorUnjamAngle);
            if (agitatorUnjamRotateTimeout.isExpired() ||
                    fabsf(connectedAgitator->getAgitatorAngle() -
                    connectedAgitator->getAgitatorDesiredAngle())
                    < AGITATOR_SETPOINT_TOLERANCE
            ) {  // either the timeout has been triggered or the agitator has reached the setpoint
                // define a random time that the agitator will take to rotate forwards.
                agitatorUnjamRotateTimeout.restart(agitatorMaxWaitTime);

                // reset the agitator
                currUnjamstate = AGITATOR_UNJAM_RESET;
            }
            break;
        }
        case AGITATOR_UNJAM_RESET:  // this is different than just agitator_rotate_command
        {
            // reset the angle to what it was before unjamming
            connectedAgitator->setAgitatorDesiredAngle(agitatorSetpointBeforeUnjam);
            // the agitator is still jammed
            if (agitatorUnjamRotateTimeout.isExpired())
            {
                // restart the timeout
                agitatorUnjamRotateTimeout.restart(agitatorMaxWaitTime);

                // define a new random angle, which will be used in the unjam back state
                // NOLINTNEXTLINE
                currAgitatorUnjamAngle = fmodf(rand(), agitatorUnjamAngleMax
                        - MIN_AGITATOR_UNJAM_ANGLE)
                        + MIN_AGITATOR_UNJAM_ANGLE;

                currAgitatorUnjamAngle = agitatorSetpointBeforeUnjam - currAgitatorUnjamAngle;

                currUnjamstate = AGITATOR_UNJAM_BACK;
            }
            else if (fabsf(connectedAgitator->getAgitatorAngle() -
                    connectedAgitator->getAgitatorDesiredAngle())
                    < AGITATOR_SETPOINT_TOLERANCE)
            {
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

// NOLINTNEXTLINE
void AgitatorUnjamCommand::end(bool)
{
    connectedAgitator->setAgitatorDesiredAngle(agitatorSetpointBeforeUnjam);
}

bool AgitatorUnjamCommand::isFinished(void) const
{
    return currUnjamstate == FINISHED;
}

}  // namespace agitator

}  // namespace aruwsrc
