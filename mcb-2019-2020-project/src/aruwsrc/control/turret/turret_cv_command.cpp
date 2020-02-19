#include "turret_cv_command.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "turret_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

TurretCVCommand::TurretCVCommand(TurretSubsystem *subsystem) :
    turretSubsystem(subsystem),
    yawTargetAngle(0, 0, 360),
    pitchTargetAngle(0, 0, 360),
    CVYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
    CVPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
{
    addSubsystemRequirement(subsystem);
}

void TurretCVCommand::execute() {
    updateTurretPosition();
}

void TurretCVCommand::pitchToEncoder(float encoder) {
    pitchTargetAngle.setValue(encoder);
    pitchTargetAngle.reboundValue();
}

void TurretCVCommand::yawToEncoder(float encoder) {
    yawTargetAngle.setValue(encoder);
    yawTargetAngle.reboundValue();
}

void TurretCVCommand::pitchIncrementEncoder(float encoder) {
    pitchTargetAngle.shiftValue(encoder);
}

void TurretCVCommand::yawIncrementEncoder(float encoder) {
    yawTargetAngle.shiftValue(encoder);
}

void TurretCVCommand::setCurve(float* pitch, float* yaw, modm::Timestamp timestamp) {
    pitchA = pitch[0];
    pitchB = pitch[1];
    pitchC = pitch[2];
    pitchD = pitch[3];
    yawA = yaw[0];
    yawB = yaw[1];
    yawC = yaw[2];
    yawD = yaw[3];
    curveBeginningTime = timestamp;
}

void TurretCVCommand::updateTurretPosition() {
    float t = modm::Clock::now().getTime() - curveBeginningTime;
    float t2 = t * t;
    float t3 = t2 * t;

    aruwlib::algorithms::ContiguousFloat goalPitch = aruwlib::algorithms::ContiguousFloat(pitchA * t3 + 
        pitchB * t2 + pitchC * t + pitchD, 0, 360);
    aruwlib::algorithms::ContiguousFloat goalYaw = aruwlib::algorithms::ContiguousFloat(yawA * t3 + 
        yawB * t2 + yawC * t + yawD, 0, 360);

    float goalPitchVel = 3 * pitchA * t2 + 2 * pitchB * t + pitchC;
    float goalYawVel = 3 * yawA * t2 + 2 * yawB * t + yawC;

    float goalPitchAcc = 6 * pitchA * t + 2 * pitchB;
    float goalYawAcc = 6 * yawA * t + 2 * pitchB;

    CVPitchPid.update(goalPitch.difference(turretSubsystem->getPitchAngle()));
    CVYawPid.update(goalYaw.difference(turretSubsystem->getYawAngle()));

    turretSubsystem->setPitchMotorOutput(CVPitchPid.getValue() + PITCH_F1 * goalPitchVel + PITCH_F2 * goalPitchAcc);
    turretSubsystem->setYawMotorOutput(CVYawPid.getValue() + YAW_F1 * goalYawVel + YAW_F2 * goalYawAcc);
}

}  // namespace control

}  // namespace aruwsrc
