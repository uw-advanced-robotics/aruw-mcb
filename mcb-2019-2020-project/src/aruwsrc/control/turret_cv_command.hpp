#ifndef __TURRET_CV_COMMAND_H__
#define __TURRET_CV_COMMAND_H__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command.hpp"
#include "src/aruwsrc/control/turret_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

class TurretCVCommand : public Command {
 public:
    explicit TurretCVCommand(void) {
        addSubsystemRequirement(&TurretSubsystem);
    }

    void initialize() {
        TurretSubsystem.turretStatus = TurretSubsystem.CV;
    }

    void execute() {
        getScaleCurve();
        updateTurretPosition();
    }

    void end(bool interrupted) {
        if (interrupted) {
            // print error message
        }
        TurretSubsystem.turretStatus = TurretSubsystem.IDLE;
    }

    bool isFinished() {
        return TurretSubsystem.turretStatus != TurretSubsystem.CV;
    }

 private:
    struct ScaleCurve {
    uint32_t timestamp;
    uint32_t timeOffset;
    // cubic coefficients
    uint32_t a_1;
    uint32_t a_2;
    uint32_t a_3;
    uint32_t a_4;
    };

    modm::Pid<float> cvYawPid;
    modm::Pid<float> cvPitchPid;

    static TurretSubsystem TurretSubsystem;

    void updateTurretPosition();

    ScaleCurve getScaleCurve();

    void followScaleCurve(aruwlib::motor::DjiMotor *motor, ScaleCurve *curve, uint32_t timestamp);
};

}  // control

}  // aruwsrc

#endif