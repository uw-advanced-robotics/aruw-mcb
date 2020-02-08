#ifndef __ENGINEER_WRIST_SUBSYSTEM_HPP__
#define __ENGINEER_WRIST_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class EngineerWristSubsystem : public Subsystem
{
 public:
    EngineerWristSubsystem();
    
    void refresh(void);

    void setWristAngleLeft(float newAngle);

    void setWristAngleRight(float newAngle);

    float getWristAngleLeft() const;

    float getWristAngleRight() const;

    float getWristDesiredAngleLeft(void) const;

    float getWristDesiredAngleRight(void) const;

    bool wristCalibrateHere(void);

 private:
   static constexpr float WRIST_GEAR_RATIO = 19.0f;
   
   static const aruwlib::motor::MotorId LEFT_MOTOR_ID = aruwlib::motor::MOTOR1;
   static const aruwlib::motor::MotorId RIGHT_MOTOR_ID = aruwlib::motor::MOTOR2;
   const aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

   aruwlib::motor::DjiMotor leftMotor;
   aruwlib::motor::DjiMotor rightMotor;

   // PID values
   const float PID_P = 80000.0f; // TODO
   const float PID_I = 0.0f;
   const float PID_D = 800000.0f;
   const float PID_MAX_ERROR_SUM = 0.0f;
   const float PID_MAX_OUTPUT = 16000;

   modm::Pid<float> leftPositionPid;
   modm::Pid<float> rightPositionPid;

   // Desired angle in radian, unwrapped
   float desiredWristAngleLeft;
   float desiredWristAngleRight;

   // Angle the wrist is initially calibrated to as a zero reference point
   float wristCalibratedAngleLeft;
   float wristCalibratedAngleRight;

   // If the wrist has been calibrated or not
   bool wristIsCalibrated;

   void wristRunPositionPid(void);

   float getUncalibratedWristAngleLeft(void) const;
   float getUncalibratedWristAngleRight(void) const;
};

}  // namespace control

}  // namespace aruwsrc

#endif //  __ENGINEER_WRIST_SUBSYSTEM_HPP__