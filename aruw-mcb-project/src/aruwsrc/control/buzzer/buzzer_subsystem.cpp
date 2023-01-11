#include "buzzer_subsystem.hpp"

#include "tap/control/subsystem.hpp"

namespace aruwsrc
{

BuzzerSubsystem::BuzzerSubsystem(aruwsrc::Drivers* drivers) : Subsystem(drivers) {}

void BuzzerSubsystem::playNoise(){
    tap::buzzer::playNote(&(drivers->pwm), 440);
}

void BuzzerSubsystem::stop(){
    tap::buzzer::silenceBuzzer(&(drivers->pwm));
}

}  // namespace aruwsrc
