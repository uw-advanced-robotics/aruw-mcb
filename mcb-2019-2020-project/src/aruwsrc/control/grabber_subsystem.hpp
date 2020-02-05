/**
 * This is part of aruw's library.
 * 
 * This is a subsystem code for Engineer grabber mechanism. 
 * The grabber will be actuated by two pneumatic pistons. 
 */

#ifndef __SUBSYSTEM_GRABBER_HPP__
#define __SUBSYSTEM_GRABBER_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/control/subsystem.hpp"

using namespace aruwlib::control;
using grabberDigitalOutPin = Board::DigitalOutPinG; 

namespace aruwsrc
{

namespace control
{

class GrabberSubsystem : public Subsystem
{
 public:
   GrabberSubsystem(): isGrabberSqueezed(false) {}

   void refresh(void);

   void setSqueezed(bool isGrabberSqueezed); 

 private: 
   bool isGrabberSqueezed; 
    
};

}  // namespace control

}  // namespace aruwsrc

#endif