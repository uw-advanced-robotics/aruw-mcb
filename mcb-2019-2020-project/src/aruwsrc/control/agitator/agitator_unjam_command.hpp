#ifndef __AGITATOR_UNJAM_COMMAND_HPP__
#define __AGITATOR_UNJAM_COMMAND_HPP__

#include <modm/processing/timer/timeout.hpp>
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/control/command.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
#include "agitator_subsystem.hpp"

namespace aruwsrc
{

namespace agitator
{

class AgitatorUnjamCommand : public aruwlib::control::Command
{
public:
    AgitatorUnjamCommand(
        AgitatorSubsystem* agitator,
        float agitatorMaxUnjamAngle,
        uint32_t agitatorMaxWaitTime = AGITATOR_MAX_WAIT_TIME
    );

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;

private:
    static constexpr float AGITATOR_SETPOINT_TOLERANCE = aruwlib::algorithms::PI / 16.0f;

    static const uint32_t AGITATOR_MAX_WAIT_TIME;

    static constexpr float MIN_AGITATOR_UNJAM_ANGLE = aruwlib::algorithms::PI / 4;;

    enum AgitatorUnjamState {AGITATOR_UNJAM_BACK, AGITATOR_UNJAM_FORWARD, FINISHED};

    AgitatorUnjamState currUnjamstate;

    // time allowed to rotate back the the currAgitatorUnjamAngle
    modm::ShortTimeout agitatorUnjamRotateTimeout;

    uint32_t agitatorMaxWaitTime;

    AgitatorSubsystem* connectedAgitator;

    float agitatorUnjamAngleMax;

    float currAgitatorUnjamAngle;

    float agitatorSetpointBeforeUnjam;
};

}  // namespace control

}  // namespace aruwsrc

#endif
