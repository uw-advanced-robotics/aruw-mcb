#ifndef __WIGGLE_DRIVE_COMMAND_HPP__
#define __WIGGLE_DRIVE_COMMAND_HPP__

#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/architecture/clock.hpp>
#include <aruwlib/control/command.hpp>

#include "aruwsrc/control/turret/turret_subsystem.hpp"

#include "chassis_subsystem.hpp"

namespace aruwsrc
{
namespace chassis
{
template <typename Drivers> class WiggleDriveCommand : public aruwlib::control::Command
{
public:
    explicit WiggleDriveCommand(
        ChassisSubsystem<Drivers>* chassis,
        aruwsrc::turret::TurretSubsystem<Drivers>* turret)
        : chassis(chassis),
          turret(turret)
    {
        addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(chassis));
    }

    void initialize() override
    {
        float turretCurAngle = turret->getYawAngleFromCenter();

        // We don't apply the sine wave if we are out of the angle of the sine wave.
        outOfCenter = fabsf(turretCurAngle) > WIGGLE_MAX_ROTATE_ANGLE;

        turretCurAngle = aruwlib::algorithms::limitVal(
            turretCurAngle,
            -WIGGLE_MAX_ROTATE_ANGLE,
            WIGGLE_MAX_ROTATE_ANGLE);

        // We use the limited current turret angle to calculate a time offset in
        // a angle vs. time graph so when we start at some angle from center we
        // are still in phase.
        startTimeForAngleOffset = asinf(turretCurAngle / WIGGLE_MAX_ROTATE_ANGLE) * WIGGLE_PERIOD /
                                  (2.0f * aruwlib::algorithms::PI);

        // The offset so when we start calculating a rotation angle, the initial
        // time is zero.
        timeOffset = aruwlib::arch::clock::getTimeMilliseconds();
    }

    void execute() override
    {
        float r;
        float x = Drivers::controlOperatorInterface.getChassisXInput() *
                  ChassisSubsystem<Drivers>::MAX_WHEEL_SPEED_SINGLE_MOTOR;
        float y = Drivers::controlOperatorInterface.getChassisYInput() *
                  ChassisSubsystem<Drivers>::MAX_WHEEL_SPEED_SINGLE_MOTOR;

        // We only wiggle when the turret is online.
        if (turret->isTurretOnline())
        {
            float curTime =
                static_cast<float>(aruwlib::arch::clock::getTimeMilliseconds() - timeOffset) -
                startTimeForAngleOffset;
            float desiredAngleError;
            float turretYawAngle = turret->getYawAngleFromCenter();

            if (outOfCenter)
            {
                outOfCenter = fabsf(turretYawAngle) > WIGGLE_MAX_ROTATE_ANGLE;
                if (!outOfCenter)
                {
                    initialize();
                }
            }

            if (!outOfCenter)
            {
                desiredAngleError = wiggleSin(curTime) + turretYawAngle;
            }
            else
            {
                desiredAngleError = aruwlib::algorithms::limitVal(
                    turretYawAngle,
                    -WIGGLE_OUT_OF_CENTER_MAX_ROTATE_ERR,
                    WIGGLE_OUT_OF_CENTER_MAX_ROTATE_ERR);
            }

            // Wrapping between -180 and 180.
            aruwlib::algorithms::ContiguousFloat rotationError(desiredAngleError, -180.0f, 180.0f);
            r = chassis->chassisSpeedRotationPID(rotationError.getValue(), WIGGLE_ROTATE_KP);
            x *= TRANSLATIONAL_SPEED_FRACTION_WHILE_WIGGLING;
            y *= TRANSLATIONAL_SPEED_FRACTION_WHILE_WIGGLING;
            // Apply a rotation matrix to the user input so you drive turret
            // relative while wiggling.
            aruwlib::algorithms::rotateVector(
                &x,
                &y,
                -aruwlib::algorithms::degreesToRadians(turretYawAngle));
        }
        else
        {
            r = Drivers::controlOperatorInterface.getChassisRInput() *
                ChassisSubsystem<Drivers>::MAX_WHEEL_SPEED_SINGLE_MOTOR;
        }

        chassis->setDesiredOutput(x, y, r);
    }

    void end(bool) override { chassis->setDesiredOutput(0.0f, 0.0f, 0.0f); }

    bool isFinished() const override { return false; }

    const char* getName() const override { return "chassis wiggle drive command"; }

private:
    static constexpr float WIGGLE_PERIOD = 1600.0f;
    static constexpr float WIGGLE_MAX_ROTATE_ANGLE = 60.0f;
    static constexpr float WIGGLE_ROTATE_KP = -250.0f;
    static constexpr float TRANSLATIONAL_SPEED_FRACTION_WHILE_WIGGLING = 0.5f;
    static constexpr float WIGGLE_OUT_OF_CENTER_MAX_ROTATE_ERR = 10.0f;

    ChassisSubsystem<Drivers>* chassis;
    aruwsrc::turret::TurretSubsystem<Drivers>* turret;

    uint32_t timeOffset = 0;
    float startTimeForAngleOffset = 0.0f;
    bool outOfCenter = false;

    // sin curve to determine angle to rotate to based on current "time"
    float wiggleSin(float t)
    {
        return WIGGLE_MAX_ROTATE_ANGLE * sinf((2.0f * aruwlib::algorithms::PI / WIGGLE_PERIOD) * t);
    }
};

}  // namespace chassis

}  // namespace aruwsrc

#endif
