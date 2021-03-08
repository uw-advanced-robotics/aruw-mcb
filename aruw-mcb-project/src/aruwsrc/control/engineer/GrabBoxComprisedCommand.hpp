#ifndef GRAB_BOX_COMPRISED_COMMAND_HPP_
#define GRAB_BOX_COMPRISED_COMMAND_HPP_

#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/control/comprised_command.hpp>

#include "EngineerWristCalibrateCommand.hpp"
#include "EngineerWristRotateCommand.hpp"
#include "EngineerWristSubsystem.hpp"
#include "squeeze_grabber_command.hpp"

namespace aruwsrc
{
namespace engineer
{
class GrabberSubsystem;
class GrabBoxComprisedCommand : public aruwlib::control::ComprisedCommand
{
public:
    static constexpr float GRAB_BIN_ANGLE = EngineerWristSubsystem::MAX_WRIST_ANGLE;
    static constexpr float DUMP_BIN_ANGLE = EngineerWristSubsystem::MIN_WRIST_ANGLE;
    static constexpr float THROW_BIN_ANGLE = 120.0f;
    static constexpr float THROW_BIN_ROTATE_TIME = 200.0f;
    static constexpr uint32_t DUMP_TIME = 1000;

    GrabBoxComprisedCommand(
        aruwlib::Drivers* drivers,
        EngineerWristSubsystem* wrist,
        GrabberSubsystem* grabber);

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;

private:
    enum class GrabberState : uint8_t
    {
        EXTENDING_GRABBER,
        RETREIVING,
        DUMPING,
        THROWING,
        FINISHED
    };

    GrabberState cmdState;

    // Grabber
    GrabberSubsystem* grabber;

    // Wrist
    EngineerWristSubsystem* wrist;
    EngineerWristRotateCommand rotateWristOut;
    EngineerWristRotateCommand rotateWristIn;
    EngineerWristRotateCommand throwBin;

    aruwlib::arch::MilliTimeout dumpingTimer;
};
}  // namespace engineer
}  // namespace aruwsrc

#endif  // GRAB_BOX_COMPRISED_COMMAND_HPP_
