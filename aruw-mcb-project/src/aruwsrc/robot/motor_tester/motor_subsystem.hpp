#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::robot::motor_tester
{
class MotorSubsystem : public tap::control::Subsystem
{
public:
    explicit MotorSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& motor);

    float getDesiredOutput() { return desiredOutput; }

    void setDesiredOutput(float out) {
        desiredOutput = std::clamp(
            out, 
            -DjiMotor::MAX_OUTPUT_C620,
            DjiMotor::MAX_OUTPUT_C620
        );

    }
    void refresh(){
        motor.setDesiredOutput(desiredOutput);
    }



private:
    float desiredOutput = 0.0;
    tap::motor::MotorInterface& motor;
};
}  // namespace aruwsrc::robot