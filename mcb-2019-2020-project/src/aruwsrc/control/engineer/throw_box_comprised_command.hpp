#ifndef __THROW_BOX_COMPRISED_COMMAND_HPP__
#define __THROW_BOX_COMPRISED_COMMAND_HPP__

#include "src/aruwlib/control/comprised_command.hpp"
#include "grabber_subsystem.hpp"
#include "squeeze_grabber_command.hpp"
#include "wrist_subsystem.hpp"
#include "wrist_calibrate_command.hpp"
#include "wrist_rotate_command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace engineer
{

class ThrowBoxComprisedCommand : public ComprisedCommand
{
 public:
   ThrowBoxComprisedCommand(
      GrabberSubsystem* grabber,
      WristSubsystem* wrist,
      float wristAngleChange,
      float wristRotateTime
   );

   void initialize();

   void execute();

   void end(bool interrupted);

   bool isFinished() const;

 private:
   // Grabber
   GrabberSubsystem* connectedGrabber;

   GrabberCommand grabberCommand;

   // Wrist
   WristSubsystem* connectedWrist;

   WristRotateCommand wristOutCommand;

   bool throwSequenceCommencing;
};

}  // namespace engineer

}  // namespace aruwsrc

#endif  // __THROW_BOX_COMPRISED_COMMAND_HPP__