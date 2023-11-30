
#include "tap/motor/dji_motor.hpp"
#include "tap/drivers.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/communication/serial/remote.hpp"


class MotorSubsystem : public tap::control::Subsystem
{
public:
    MotorSubsystem(tap::Drivers* drivers, tap::motor::MotorInterface& motor, tap::communication::serial::Remote::Channel channel, int16_t maxOutput)
        : Subsystem(drivers), motor(motor), channel(channel), maxOutput(maxOutput) {}
    
    inline void initialize() override { this->motor.initialize(); };
    inline void refresh() override { this->motor.setDesiredOutput(static_cast<int>(this->drivers->remote.getChannel(this->channel) * maxOutput)); };
    inline void refreshSafeDisconnect() override { this->motor.setDesiredOutput(0); };
    const char* getName() override { return "Motor"; }
private:
    tap::motor::MotorInterface& motor;
    tap::communication::serial::Remote::Channel channel;
    int16_t maxOutput;
};