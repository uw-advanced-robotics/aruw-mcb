/**
 * This is part of aruw's library.
 * 
 * This is a subsystem code for y-axis movement. Connect this to
 * a digital output pin (preset to pin E on our board). This
 * sends a digital out signal to a solenoid, which actuates
 * a piston, used for collecting far bins.
 */

#ifndef __SUBSYSTEM_YAXIS_HPP__
#define __SUBSYSTEM_YAXIS_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/control/subsystem.hpp"

using namespace aruwlib::control;

using yAxisDigitalOutPin = Board::DigitalOutPinE;

namespace aruwsrc
{

namespace engineer
{

class YAxisSubsystem : public Subsystem
{
 public:
    YAxisSubsystem(): isExtended(false) {}

    void refresh();

    void setYAxisExtended(bool isExtended);

    bool getIsExtended();

 private:
    bool isExtended;
};

}  // namespace engineer

}  // namespace aruwsrc

#endif
