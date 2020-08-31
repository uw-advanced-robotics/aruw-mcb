#ifndef __AGITATOR_ROTATE_COMMAND_HPP__
#define __AGITATOR_ROTATE_COMMAND_HPP__

#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/algorithms/ramp.hpp>
#include <aruwlib/architecture/clock.hpp>
#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/control/command.hpp>
#include <modm/math/filter/pid.hpp>

#include "agitator_subsystem.hpp"

namespace aruwsrc
{
namespace agitator
{
/**
 * Rotates the connected agitator some angle in some desired time. Currently
 * pass in a rotate velocity and it uses modm::Clock::now() to determine the
 * proper ramp increment.
 */
template <typename Drivers>
class AgitatorRotateCommand : public aruwlib::control::Command<Drivers>
{
public:
    /**
     * @param agitator the agitator associated with the rotate command
     * @param agitatorAngleChange the desired rotation angle
     * @param agitatorRotateTime the time it takes to rotate the agitator to the desired angle
     *                           in milliseconds
     * @param setpointTolerance the angle difference between current and desired angle when the
     *                          command will be considered to be completed (used in isFinished
     *                          function). Only set this if you want a different tolerance,
     *                          otherwise the above tolerance is usually fine.
     * @attention the ramp value is calculated by finding the rotation speed
     *            (agitatorAngleChange / agitatorRotateTime), and then multiplying this by
     *            the period (how often the ramp is called)
     */
    AgitatorRotateCommand(
        AgitatorSubsystem<Drivers>* agitator,
        float agitatorAngleChange,
        uint32_t agitatorRotateTime,
        uint32_t agitatorPauseAfterRotateTime,
        bool agitatorSetToFinalAngle,
        float setpointTolerance = AGITATOR_SETPOINT_TOLERANCE)
        : connectedAgitator(agitator),
          agitatorTargetAngleChange(agitatorAngleChange),
          rampToTargetAngle(0.0f),
          agitatorDesiredRotateTime(agitatorRotateTime),
          agitatorMinRotatePeriod(agitatorRotateTime + agitatorPauseAfterRotateTime),
          agitatorMinRotateTimeout(agitatorRotateTime + agitatorPauseAfterRotateTime),
          agitatorSetpointTolerance(setpointTolerance),
          agitatorPrevRotateTime(0),
          agitatorSetToFinalAngle(agitatorSetToFinalAngle)
    {
        this->addSubsystemRequirement(
            dynamic_cast<aruwlib::control::Subsystem<Drivers>*>(agitator));
    }

    const char* getName() const override { return "agitator rotate command"; }

    void initialize() override
    {
        // set the ramp start and target angles
        rampToTargetAngle.setTarget(
            connectedAgitator->getAgitatorDesiredAngle() + agitatorTargetAngleChange);

        // reset unjam and min rotate timeouts
        connectedAgitator->armAgitatorUnjamTimer(agitatorMinRotatePeriod);
        agitatorMinRotateTimeout.restart(agitatorMinRotatePeriod);

        agitatorPrevRotateTime = aruwlib::arch::clock::getTimeMilliseconds();
    }

    void execute() override
    {
        // update the agitator setpoint ramp
        uint32_t currTime = aruwlib::arch::clock::getTimeMilliseconds();
        rampToTargetAngle.update(
            (currTime - agitatorPrevRotateTime) * agitatorTargetAngleChange /
            static_cast<float>(agitatorDesiredRotateTime));
        agitatorPrevRotateTime = currTime;
        connectedAgitator->setAgitatorDesiredAngle(rampToTargetAngle.getValue());
    }

    void end(bool) override
    {
        // if the agitator is not interrupted, then it exited normally
        // (i.e. reached the desired angle) and is not jammed. If it is
        // jammed we thus want to set the agitator angle to the current angle,
        // so the motor does not attempt to keep rotating forward (and possible stalling)
        if (connectedAgitator->isAgitatorJammed())
        {
            connectedAgitator->setAgitatorDesiredAngle(connectedAgitator->getAgitatorAngle());
        }
        else if (agitatorSetToFinalAngle)
        {
            connectedAgitator->setAgitatorDesiredAngle(rampToTargetAngle.getTarget());
        }
        else
        {
            connectedAgitator->setAgitatorDesiredAngle(connectedAgitator->getAgitatorAngle());
        }
        connectedAgitator->disarmAgitatorUnjamTimer();
    }

    bool isFinished() const override
    {
        // The agitator is within the setpoint tolerance, the agitator ramp is
        // finished, and the minimum rotate time is expired.
        return fabsf(
                   connectedAgitator->getAgitatorAngle() -
                   connectedAgitator->getAgitatorDesiredAngle()) < agitatorSetpointTolerance &&
               agitatorMinRotateTimeout.isExpired();
    }

private:
    static constexpr float AGITATOR_SETPOINT_TOLERANCE = aruwlib::algorithms::PI / 16.0f;

    AgitatorSubsystem<Drivers>* connectedAgitator;

    float agitatorTargetAngleChange;

    aruwlib::algorithms::Ramp rampToTargetAngle;

    // time you want the agitator to take to rotate to the desired angle, in milliseconds
    uint32_t agitatorDesiredRotateTime;

    uint32_t agitatorMinRotatePeriod;

    aruwlib::arch::MilliTimeout agitatorMinRotateTimeout;

    float agitatorSetpointTolerance;

    uint32_t agitatorPrevRotateTime;

    bool agitatorSetToFinalAngle;
};

}  // namespace agitator

}  // namespace aruwsrc

#endif
