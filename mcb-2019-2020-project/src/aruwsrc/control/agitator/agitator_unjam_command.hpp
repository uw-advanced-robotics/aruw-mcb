#ifndef __AGITATOR_UNJAM_COMMAND_HPP__
#define __AGITATOR_UNJAM_COMMAND_HPP__

#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/control/command.hpp>
#include <aruwlib/motor/dji_motor.hpp>

#include "agitator_subsystem.hpp"

namespace aruwsrc
{
namespace agitator
{
template <typename Drivers> class AgitatorUnjamCommand : public aruwlib::control::Command<Drivers>
{
public:
    AgitatorUnjamCommand(
        AgitatorSubsystem<Drivers>* agitator,
        float agitatorMaxUnjamAngle,
        uint32_t agitatorMaxWaitTime = AGITATOR_MAX_WAIT_TIME)
        : currUnjamstate(AGITATOR_UNJAM_BACK),
          agitatorUnjamRotateTimeout(0),
          salvationTimeout(0),
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
        this->addSubsystemRequirement(
            dynamic_cast<aruwlib::control::Subsystem<Drivers>*>(agitator));
        salvationTimeout.stop();
        agitatorUnjamRotateTimeout.stop();
    }

    void initialize() override
    {
        // define a random time that the agitator will take to unjam backwards.
        agitatorUnjamRotateTimeout.restart(agitatorMaxWaitTime);

        // define a random unjam angle between [MIN_AGITATOR_UNJAM_ANGLE, agitatorUnjamAngleMax]

        float randomUnjamAngle = fmodf(rand(), agitatorUnjamAngleMax - MIN_AGITATOR_UNJAM_ANGLE) +
                                 MIN_AGITATOR_UNJAM_ANGLE;

        // subtract this angle from the current angle to rotate agitator backwards
        currAgitatorUnjamAngle = connectedAgitator->getAgitatorAngle() - randomUnjamAngle;

        // store the current setpoint angle to be referenced later
        agitatorSetpointBeforeUnjam = connectedAgitator->getAgitatorDesiredAngle();
        currUnjamstate = AGITATOR_UNJAM_BACK;

        salvationTimeout.restart(SALVATION_TIMEOUT_MS);
    }

    void execute() override
    {
        if (salvationTimeout.execute())
        {
            currAgitatorUnjamAngle = agitatorSetpointBeforeUnjam - 2 * aruwlib::algorithms::PI;
            salvationTimeout.stop();
            agitatorUnjamRotateTimeout.restart(SALVATION_UNJAM_BACK_WAIT_TIME);
            currUnjamstate = AGITATOR_SALVATION_UNJAM_BACK;
        }

        switch (currUnjamstate)
        {
            case AGITATOR_SALVATION_UNJAM_BACK:
            {
                connectedAgitator->setAgitatorDesiredAngle(currAgitatorUnjamAngle);
                if (agitatorUnjamRotateTimeout.isExpired() ||
                    fabsf(
                        connectedAgitator->getAgitatorAngle() -
                        connectedAgitator->getAgitatorDesiredAngle()) < AGITATOR_SETPOINT_TOLERANCE)
                {
                    currUnjamstate = FINISHED;
                }
                break;
            }
            case AGITATOR_UNJAM_BACK:
            {
                connectedAgitator->setAgitatorDesiredAngle(currAgitatorUnjamAngle);
                if (agitatorUnjamRotateTimeout.isExpired() ||
                    fabsf(
                        connectedAgitator->getAgitatorAngle() -
                        connectedAgitator->getAgitatorDesiredAngle()) < AGITATOR_SETPOINT_TOLERANCE)
                {  // either the timeout has been triggered or the agitator has reached the setpoint
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
                    float randomUnjamAngle =
                        fmodf(rand(), agitatorUnjamAngleMax - MIN_AGITATOR_UNJAM_ANGLE) +
                        MIN_AGITATOR_UNJAM_ANGLE;

                    currAgitatorUnjamAngle = agitatorSetpointBeforeUnjam - randomUnjamAngle;

                    currUnjamstate = AGITATOR_UNJAM_BACK;
                }
                else if (
                    fabsf(
                        connectedAgitator->getAgitatorAngle() -
                        connectedAgitator->getAgitatorDesiredAngle()) < AGITATOR_SETPOINT_TOLERANCE)
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

    void end(bool) override {}

    bool isFinished() const override { return currUnjamstate == FINISHED; }

    const char* getName() const override { return "agitator unjam command"; }

private:
    static constexpr uint32_t SALVATION_TIMEOUT_MS = 2000;

    static constexpr uint32_t SALVATION_UNJAM_BACK_WAIT_TIME = 1000;

    static constexpr float AGITATOR_SETPOINT_TOLERANCE = aruwlib::algorithms::PI / 16.0f;

    // the maximum time that the command will wait from commanding the agitator to rotate
    // backwards to rotating forwards again.
    static constexpr uint32_t AGITATOR_MAX_WAIT_TIME = 130;

    // minimum angle the agitator will rotate backwards when unjamming
    static constexpr float MIN_AGITATOR_UNJAM_ANGLE = aruwlib::algorithms::PI / 4.0f;

    enum AgitatorUnjamState
    {
        AGITATOR_SALVATION_UNJAM_BACK,
        AGITATOR_UNJAM_BACK,
        AGITATOR_UNJAM_RESET,
        FINISHED
    };

    AgitatorUnjamState currUnjamstate;

    // time allowed to rotate back the the currAgitatorUnjamAngle
    aruwlib::arch::MilliTimeout agitatorUnjamRotateTimeout;

    aruwlib::arch::MilliTimeout salvationTimeout;

    // usually set to AGITATOR_MAX_WAIT_TIME, but can be user enabled
    uint32_t agitatorMaxWaitTime;

    AgitatorSubsystem<Drivers>* connectedAgitator;

    float agitatorUnjamAngleMax;

    float currAgitatorUnjamAngle;

    float agitatorSetpointBeforeUnjam;
};

}  // namespace agitator

}  // namespace aruwsrc

#endif
