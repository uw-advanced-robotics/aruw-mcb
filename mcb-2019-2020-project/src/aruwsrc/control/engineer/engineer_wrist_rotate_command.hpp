#ifndef __ENGINEER_WRIST_ROTATE_COMMAND_HPP__
#define __ENGINEER_WRIST_ROTATE_COMMAND_HPP__

#include <modm/math/filter/pid.hpp>
#include <modm/math/filter/ramp.hpp>
#include "src/aruwlib/control/command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "engineer_wrist_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

class EngineerWristRotateCommand : Command
{
 public:
    static const uint32_t WRIST_MIN_ROTATE_TIME = 300;

    EngineerWristRotateCommand(
        EngineerWristSubsystem* wrist,
        float wristAngleChange,
        float wristRotateTime
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
    static constexpr float WRIST_SETPOINT_TOLERANCE = aruwlib::algorithms::PI / 30.0f;

    static constexpr float WRIST_ROTATE_COMMAND_PERIOD = 3;

    EngineerWristSubsystem* connectedWrist;

    float wristTargetChange;

    modm::filter::Ramp<float> wristRotateSetpointLeft;
    modm::filter::Ramp<float> wristRotateSetpointRight;

    float wristDesiredRotateTime;

    modm::ShortTimeout wristMinRotateTime;
};

}  // namespace control

}  // namespace aruwsrc

#endif  // __ENGINEER_WRIST_IN_COMMAND_HPP__