#ifndef __TURRET_INIT_COMMAND_H__
#define __TURRET_INIT_COMMAND_H__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class TurretSubsystem;
class TurretInitCommand : public Command {
 public:
    explicit TurretInitCommand(TurretSubsystem *subsystem);

    void initialize() {}
    bool isFinished() const;

    void execute();
    void end(bool interrupted) { if (interrupted) { return; } }

 private:
    uint16_t YAW_P = 1.0f;
    uint16_t YAW_I = 0.0f;
    uint16_t YAW_D = 0.0f;
    uint16_t YAW_MAX_ERROR_SUM = 0.0f;
    uint16_t YAW_MAX_OUTPUT = 16000;

    uint16_t PITCH_P = 1.0f;
    uint16_t PITCH_I = 0.0f;
    uint16_t PITCH_D = 0.0f;
    uint16_t PITCH_MAX_ERROR_SUM = 0.0f;
    uint16_t PITCH_MAX_OUTPUT = 16000;

    const int TURRET_START_ANGLE = 90;

    TurretSubsystem *turretSubsystem;

    modm::Pid<float> initYawPid;
    modm::Pid<float> initPitchPid;

    void updateTurretPosition(void);
};

}  // namespace control

}  // namespace aruwsrc

#endif
