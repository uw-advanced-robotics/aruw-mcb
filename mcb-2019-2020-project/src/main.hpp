#ifndef __MAIN_HPP__  // todo is this weird
#define __MAIN_HPP__

#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/communication/serial/ref_serial.hpp"

extern aruwlib::control::CommandScheduler mainScheduler(true);
extern aruwlib::serial::RefSerial refSerial;

#endif

