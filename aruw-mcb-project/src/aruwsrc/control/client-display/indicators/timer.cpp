#include "timer.hpp"

#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
    long startTime;
    Timer::Timer(RefSerialTransmitter &refSerialTransmitter, const RefSerial &refSerial)
        :HudIndicator(refSerialTransmitter),
        numberIndicator(refSerialTransmitter, &numberGraphic, updateTimer, 0),
        refSerial(refSerial){
            startTime = tap::arch::clock::getTimeMilliseconds() / 1000;
        }


        modm::ResumableResult<void> Timer::update(){
            timer = (tap::arch::clock::getTimeMilliseconds() / 1000) - startTime;
        }
}