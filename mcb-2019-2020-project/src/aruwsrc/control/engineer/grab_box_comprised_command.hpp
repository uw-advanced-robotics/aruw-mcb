#ifndef __GRAB_BOX_COMPRISED_COMMAND_HPP__
#define __GRAB_BOX_COMPRISED_COMMAND_HPP__

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

class GrabBoxComprisedCommand : public ComprisedCommand
{
 public:
   GrabBoxComprisedCommand(
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

   WristRotateCommand wristInCommand;

   WristRotateCommand wristOutCommand;

   enum states {out, in, done};
   states ps;
};

}  // namespace engineer

}  // namespace aruwsrc

#endif  // __GRAB_BOX_COMPRISED_COMMAND_HPP__