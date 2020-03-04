#ifndef __DUMP_BOX_COMPRISED_COMMAND_HPP__
#define __DUMP_BOX_COMPRISED_COMMAND_HPP__

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

class DumpBoxComprisedCommand : public ComprisedCommand
{
 public:
   DumpBoxComprisedCommand(
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

   bool dumpSequenceCommencing;
};

}  // namespace engineer

}  // namespace aruwsrc

#endif  // __DUMP_BOX_COMPRISED_COMMAND_HPP__