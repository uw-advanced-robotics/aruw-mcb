#ifndef __DRONE_TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP__
#define __DRONE_TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP__

#include <aruwsrc/algorithms/turret_pid.hpp>
#include <aruwlib/control/command.hpp>
#include <aruwlib/algorithms/contiguous_float.hpp>
#include <aruwsrc/control/turret/drone_turret_subsystem.hpp>
#include <aruwlib/communication/sensors/bno055/uart_bno055.hpp>
extern aruwlib::sensors::UartBno055<Uart7> bno055;
extern aruwlib::sensors::Bno055Data bno055Data;
namespace aruwsrc
{

namespace turret
{

class DroneTurretWorldRelativePositionCommand : public Command
{
 public:
    DroneTurretWorldRelativePositionCommand(DroneTurretSubsystem *subsystem);

    void initialize();

    bool isFinished() const {return false;}

    void execute();

    void end(bool);

    void refresh();

 private:
    static constexpr float YAW_P = 1000.0f;
    static constexpr float YAW_I = 0.0f;
    static constexpr float YAW_D = 40.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
    static constexpr float YAW_MAX_OUTPUT = 7000.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

    static constexpr float PITCH_P = 1000.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 40.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 7000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.5f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;

    static constexpr float USER_YAW_INPUT_SCALAR = 0.75f;
    static constexpr float USER_PITCH_INPUT_SCALAR = - 0.5f;

    static constexpr float PITCH_GRAVITY_COMPENSATION_KP = 2700.0f;

    DroneTurretSubsystem *turretSubsystem;

    aruwlib::algorithms::ContiguousFloat yawTargetAbsoluteAngle;
    aruwlib::algorithms::ContiguousFloat pitchTargetAbsoluteAngle;

    aruwlib::algorithms::ContiguousFloat absoluteYawAngle;
    aruwlib::algorithms::ContiguousFloat absolutePitchAngle;

    float imuInitialYaw;
    float imuInitialPitch;

    aruwsrc::algorithms::TurretPid yawPid;
    aruwsrc::algorithms::TurretPid pitchPid;

    void runYawPositionController();
    void runPitchPositionController();

   static float projectChassisRelativeYawToWorldRelative(float yawAngle, float imuInitialAngle);
   static float projectWorldRelativeYawToChassisFrame(float yawAngle, float imuInitialAngle);

   static float projectChassisRelativePitchToWorldRelative(float pitchAngle, float imuInitialAngle);
   static float projectWorldRelativePitchToChassisFrame(float pitchAngle, float imuInitialAngle);

   // float calculateYawFeedforward(float baseYaw, float basePitch, float baseRoll, float turretYaw) {
      
   // }

   // float calculatePitchFeedforward(float baseYaw, float basePitch, float baseRoll, float turretYaw, float turretPitch) {
      
   // }

};

}  // namespace turret

}  // namespace aruwsrc

#endif
