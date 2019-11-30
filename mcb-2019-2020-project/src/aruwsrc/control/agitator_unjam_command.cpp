#include "agitator_unjam_command.hpp"

namespace aruwsrc
{

namespace control
{

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
    agitatorSetpointBeforeUnjam(0),
    unjamComplete(false)
{
    agitatorUnjamRotateTimeout.stop();
}

void AgitatorUnjamCommand::initialize()
{
    // define a random time that the agitator will take to unjam.
    modm::ShortTimestamp randomUnjamTime = (rand() % agitatorMaxWaitTime + MIN_AGITATOR_MAX_WAIT_TIME)
        % agitatorMaxWaitTime;
    agitatorUnjamRotateTimeout.restart(randomUnjamTime);

    // define a random unjam angle
    currAgitatorUnjamAngle = fmod((fmod(static_cast<float>(rand()), agitatorUnjamAngleMax) +
        MIN_AGITATOR_UNJAM_ANGLE), agitatorUnjamAngleMax);

    // subtract this angle from the current angle
    currAgitatorUnjamAngle -= connectedAgitator->getAgitatorEncoderToPosition();

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
                fabs(connectedAgitator->getAgitatorEncoderToPosition() -
                connectedAgitator->getAgitatorDesiredAngle()) < agitatorSetpointTolerance
            ) {
                // define a random time that the agitator will take to unjam.
                modm::ShortTimestamp randomUnjamTime = (rand() % agitatorMaxWaitTime + MIN_AGITATOR_MAX_WAIT_TIME)
                    % agitatorMaxWaitTime;
                agitatorUnjamRotateTimeout.restart(randomUnjamTime);

                // move on to unjam forward state
                currUnjamstate = AGITATOR_UNJAM_FORWARD;
            }
            break;
        }
        case AGITATOR_UNJAM_FORWARD:  // this is inherently different than just agitator_rotate_command
        {
            connectedAgitator->setAgitatorAngle(agitatorSetpointBeforeUnjam);
            if (agitatorUnjamRotateTimeout.isExpired())
            {
                currUnjamstate = AGITATOR_UNJAM_BACK;
            }
            else if (
                fabs(connectedAgitator->getAgitatorEncoderToPosition() -
                connectedAgitator->getAgitatorDesiredAngle()) < agitatorSetpointTolerance
            ) {
                currUnjamstate = FINISHED;
            }
            break;
        }
        case FINISHED:  // this could be only two states, but its simpler to understand and debug with three
        {
            unjamComplete = true;
            break;
        }
    }
}

void AgitatorUnjamCommand::end(bool interrupted)
{
    connectedAgitator->setAgitatorAngle(agitatorSetpointBeforeUnjam);
}

bool AgitatorUnjamCommand::isFinished(void) const
{
    return unjamComplete;
}

}  // namespace control

}  // namespace aruwsrc
