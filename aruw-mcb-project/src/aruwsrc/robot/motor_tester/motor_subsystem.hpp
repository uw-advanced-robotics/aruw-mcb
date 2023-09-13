#include "tap/motor/dji_motor.hpp"
#include "tap/control/subsystem.hpp"


class MotorSubsystem : public tap::control::Subsystem
{
public:
    inline MotorSubsystem(tap::Drivers& drivers, tap::motor::DjiMotor& motor) : Subsystem(&drivers), motor(motor) {}
    inline void initialize() override final
    {
        this->motor.initialize();
    }
    inline void refresh() override final {}
    inline void refreshSafeDisconnect() override final
    {
        this->motor.setDesiredOutput(0);
    }

    inline void setMotorOutput(float desiredOutput)
    {
        this->motor.setDesiredOutput(desiredOutput);
    }
private:
    tap::motor::DjiMotor& motor;
};