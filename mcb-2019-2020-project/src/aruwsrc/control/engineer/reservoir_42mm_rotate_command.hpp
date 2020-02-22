#ifndef __RESERVOIR_42MM_ROTATE_COMMAND__
#define __RESERVOIR_42MM_ROTATE_COMMAND__

#include <modm/math/filter/pid.hpp>
#include <modm/math/filter/ramp.hpp>
#include "src/aruwlib/control/command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "reservoir_42mm_subsystem.hpp"

namespace aruwsrc
{

namespace engineer
{

class Reservoir42mmRotateCommand : public aruwlib::control::Command
{
 public:
    Reservoir42mmRotateCommand(
        Reservoir42mmSubsystem* reservoir,
        float reservoirAngleChange,
        float reservoirRotateTime
    );

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;

 private:
    static constexpr float RESERVOIR_SETPOINT_TOLERANCE = aruwlib::algorithms::PI / 30.0f;
    
    static constexpr float RESERVOIR_ROTATE_COMMAND_PERIOD = 3;

    Reservoir42mmSubsystem* connectedReservoir;

    float reservoirTargetChange;

    modm::filter::Ramp<float> reservoirRotateSetpoint;

    float reservoirDesiredRotateTime;

    modm::ShortTimeout reservoirMinRotateTime;
};

}  // namespace engineer

}  // namespace aruwsrc

#endif  // __RESERVOIR_42MM_ROTATE_COMMAND__