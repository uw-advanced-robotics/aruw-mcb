#include "EngineerWristCalibrateCommand.hpp"

#include <aruwlib/control/subsystem.hpp>

#include "EngineerWristSubsystem.hpp"

namespace aruwsrc
{
namespace engineer
{
EngineerWristCalibrateCommand::EngineerWristCalibrateCommand(EngineerWristSubsystem* wrist)
    : wrist(wrist)
{
    this->addSubsystemRequirement(reinterpret_cast<aruwlib::control::Subsystem*>(wrist));
}

void EngineerWristCalibrateCommand::initialize() { wrist->wristCalibrateHere(); }

void EngineerWristCalibrateCommand::execute() { wrist->wristCalibrateHere(); }

void EngineerWristCalibrateCommand::end(bool) {}

bool EngineerWristCalibrateCommand::isFinished() const { return wrist->wristCalibrateHere(); }
}  // namespace engineer
}  // namespace aruwsrc
