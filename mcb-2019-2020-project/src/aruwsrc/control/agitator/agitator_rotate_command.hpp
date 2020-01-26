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
    static const uint32_t AGITATOR_MIN_ROTATE_TIME = 300;

    AgitatorRotateCommand(
        AgitatorSubsystem* agitator,
        float agitatorAngleChange,
        float agitatorRotateTime
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

    modm::ShortTimeout agitatorMinRotateTime;
};

}  // namespace control

}  // namespace aruwsrc

#endif