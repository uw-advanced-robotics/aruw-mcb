#ifndef __WRIST_ROTATE_COMMAND_HPP__
#define __WRIST_ROTATE_COMMAND_HPP__

#include <modm/math/filter/pid.hpp>
#include <modm/math/filter/ramp.hpp>
#include "src/aruwlib/control/command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "wrist_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

class WristRotateCommand : public aruwlib::control::Command
{
 public:
    static const uint32_t WRIST_MIN_ROTATE_TIME = 300;
    
    WristRotateCommand(
        WristSubsystem* wrist,
        float wristAngleChange,
        float wristRotateTime
    );

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;

 private:
    // TODO: should possible make the tolerance much smaller as there was some drift.
    static constexpr float WRIST_SETPOINT_TOLERANCE = aruwlib::algorithms::PI / 30.0f;

    static constexpr float WRIST_ROTATE_COMMAND_PERIOD = 3;

    WristSubsystem* connectedWrist;

    float wristTargetChange;

    modm::filter::Ramp<float> wristRotateSetpointLeft;
    modm::filter::Ramp<float> wristRotateSetpointRight;

    float wristDesiredRotateTime;

    modm::ShortTimeout wristMinRotateTime;
};

}  // namespace control

}  // namespace aruwsrc

#endif  // __WRIST_ROTATE_COMMAND_HPP__