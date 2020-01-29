#ifndef __TURRET_MANUAL_COMMAND_H__
#define __TURRET_MANUAL_COMMAND_H__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command.hpp"
//#include "src/aruwsrc/control/turret_subsystem.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class TurretSubsystem;

class TurretManualCommand : public Command {
 public:
    explicit TurretManualCommand(TurretSubsystem *turret = nullptr);

    void initialize(void);

    void execute(void);

    void end(bool interrupted);

    bool isFinished(void) const;

 private:
    uint16_t YAW_P = 0.0f;
    uint16_t YAW_I = 0.0f;
    uint16_t YAW_D = 0.0f;

    uint16_t PITCH_P = 0.0f;
    uint16_t PITCH_I = 0.0f;
    uint16_t PITCH_D = 0.0f;

    uint16_t remoteControlScaler = 0.5;

    modm::Pid<float>::Parameter *manualYawPid;
    modm::Pid<float>::Parameter *manualPitchPid;

    TurretSubsystem *turretSubsystem;

    void updateTurretPosition(void);
};

}  // control

}  // aruwsrc

#endif