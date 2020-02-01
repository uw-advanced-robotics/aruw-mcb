#ifndef __ENGINEER_17MM_RESERVOIR_ROTATE_COMMAND_HPP__
#define __ENGINEER_17MM_RESERVOIR_ROTATE_COMMAND_HPP__

#include <modm/math/filter/pid.hpp>
#include <modm/math/filter/ramp.hpp>
#include "src/aruwlib/control/command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "engineer_17mm_reservoir_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

class Engineer17mmReservoirRotateCommand : aruwlib::control::Command
{
 public:
    static const uint32_t RESERVOIR_MIN_ROTATE_TIME = 300;

    Engineer17mmReservoirRotateCommand(
        Engineer17mmReservoirSubsystem* reservoir,
        float reservoirAngleChange,
        float reservoirRotateTime
    );

    /**
     * The initial subroutine of a command.  Called once when the command is
     * initially scheduled.
     */
    void initialize();

    /**
     * The main body of a command.  Called repeatedly while the command is
     * scheduled.
     */
    void execute();

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
    static constexpr float RESERVOIR_SETPOINT_TOLERANCE = aruwlib::algorithms::PI / 30.0f;

    static constexpr float RESERVOIR_ROTATE_COMMAND_PERIOD = 3;

    Engineer17mmReservoirSubsystem* connectedReservoir;

    float reservoirTargetChange;

    modm::filter::Ramp<float> reservoirRotateSetpoint;

    float reservoirDesiredRotateTime;

    modm::ShortTimeout reservoirMinRotateTime;
};

}  // namespace control

}  // namespace aruwsrc

#endif  // __ENGINEER_17MM_RESERVOIR_ROTATE_COMMAND_HPP__