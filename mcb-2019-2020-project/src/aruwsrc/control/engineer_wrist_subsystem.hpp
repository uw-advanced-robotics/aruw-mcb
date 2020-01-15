#ifndef __ENGINEER_WRIST_SUBSYSTEM_HPP__
#define __ENGINEER_WRIST_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class EngineerWristSubsystem : public Subsystem
{
 public:
    EngineerWristSubsystem(
        aruwlib::motor::MotorId leftMotorId = LEFT_MOTOR_ID,
        aruwlib::motor::MotorId rightMotorId = RIGHT_MOTOR_ID
    );
        
    
    void refresh(void);

    void setWristAngle(float newAngle);

    // needed?

    float getWristEncoderToPosition(void) const;

    float getWristDesiredAngle(void) const;
    
    bool wristCalibrateHere(void);

 private:
    static const aruwlib::motor::MotorId LEFT_MOTOR_ID = aruwlib::motor::MOTOR1;
    static const aruwlib::motor::MotorId RIGHT_MOTOR_ID = aruwlib::motor::MOTOR2;
    const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    // from the perspective of the grabber, i.e. one will be inverted
    // motors
    aruwlib::motor::DjiMotor leftMotor;
    aruwlib::motor::DjiMotor rightMotor;

    // PID values
    const float PID_P = 10.0f;
    const float PID_I = 0.0f;
    const float PID_D = 0.0f;
    const float PID_MAX_ERROR_SUM = 0.0f;
    const float PID_MAX_OUTPUT = 16000;

    modm::Pid<float> leftPositionPid;
    modm::Pid<float> rightPositionPid;

    float desiredWristAngle;

    float wristCalibrationAngle;

    bool wristIsCalibrated;

    float wristGearRatio;

    void wristRunPositionPid(void);
};

}  // namespace control

}  // namespace aruwsrc

#endif //  __ENGINEER_WRIST_SUBSYSTEM_HPP__