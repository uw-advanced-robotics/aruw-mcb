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
    static const uint32_t AGITATOR_MIN_ROTATE_TIME = 200;

    AgitatorRotateCommand(
        AgitatorSubsystem* agitator,
        float agitatorAngleChange,
        float agitatorRotateTime
    );

    /**
     * The initial subroutine of a command.  Called once when the command is
     * initially scheduled.
     */
    void initialize(void);

    /**
     * The main body of a command.  Called repeatedly while the command is
     * scheduled.
     */
    void execute(void);

    /**
     * The action to take when the command ends.  Called when either the command
     * finishes normally, or when it interrupted/canceled.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    void end(bool interrupted);

    /**
     * Whether the command has finished.  Once a command finishes, the scheduler
     * will call its end() method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    bool isFinished() const;

 private:
    static constexpr float AGITATOR_SETPOINT_TOLERANCE = aruwlib::algorithms::PI / 16.0f;

    // how often this command is called, in seconds
    static constexpr float AGITATOR_ROTATE_COMMAND_PERIOD = 0.002f;

    AgitatorSubsystem* connectedAgitator;

    float agitatorTargetChange;

    modm::filter::Ramp<float> agitatorRotateSetpoint;

    // time you want the agitator to take to rotate to the desired angle, in seconds
    float agitatorDesiredRotateTime;

    modm::ShortTimeout agitatorMinRotateTime;
};

}  // namespace control

}  // namespace aruwsrc

#endif