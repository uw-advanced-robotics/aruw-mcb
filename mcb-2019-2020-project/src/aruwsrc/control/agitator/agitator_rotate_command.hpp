#ifndef __AGITATOR_ROTATE_COMMAND_HPP__
#define __AGITATOR_ROTATE_COMMAND_HPP__

#include <modm/math/filter/pid.hpp>
#include <modm/math/filter/ramp.hpp>
#include "src/aruwlib/control/command.hpp"
#include "agitator_subsystem.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

namespace aruwsrc
{

namespace agitator
{

class AgitatorRotateCommand : public aruwlib::control::Command
{
 public:
    /**
     * @param agitator the agitator associated with the rotate command
     * @param agitatorAngleChange the desired rotation angle
     * @param agitatorRotateTime the time it takes to rotate the agitator to the desired angle
     *                           in milliseconds
     * 
     * @attention the ramp value is calculated by finding the rotation speed
     *            (agitatorAngleChange / agitatorRotateTime), and then multiplying this by
     *            the period (how often the ramp is called)
     */
    AgitatorRotateCommand(
        AgitatorSubsystem* agitator,
        float agitatorAngleChange,
        float agitatorRotateTime,
        float agitatorPauseAfterRotateTime
    );

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;

 private:
    static constexpr float AGITATOR_SETPOINT_TOLERANCE = aruwlib::algorithms::PI / 16.0f;

    // how often this command is called, in milliseconds
    static constexpr float AGITATOR_ROTATE_COMMAND_PERIOD = 2;

    AgitatorSubsystem* connectedAgitator;

    float agitatorTargetChange;

    modm::filter::Ramp<float> agitatorRotateSetpoint;

    // time you want the agitator to take to rotate to the desired angle, in milliseconds
    float agitatorDesiredRotateTime;

    float agitatorMinRotatePeriod;

    modm::ShortTimeout agitatorMinRotateTimeout;
};

}  // namespace control

}  // namespace aruwsrc

#endif