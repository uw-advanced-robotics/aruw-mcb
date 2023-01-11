#include "tap/communication/gpio/pwm.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/drivers.hpp"

#ifndef BUZZER_SUBSYSTEM_H_
#define BUZZER_SUBSYSTEM_H_

namespace aruwsrc
{
class BuzzerSubsystem : public tap::control::Subsystem
{
public:
    BuzzerSubsystem(aruwsrc::Drivers* drivers);

    const char* getName() override { return "Buzzer"; }


    void playNoise();

	void stop();
};

}  // namespace aruwsrc

#endif