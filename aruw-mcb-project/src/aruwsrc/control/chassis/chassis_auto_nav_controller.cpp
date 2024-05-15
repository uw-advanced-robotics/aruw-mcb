#include "chassis_auto_nav_controller.hpp"

namespace aruwsrc::chassis
{
    void ChassisAutoNavController::runController(const uint32_t dt, Position currentPos) {
        // // get current position
        // Position setPoint = path.setInterpolatedPoint(currentPos);
        //TODO: ...

        if (yawMotor.isOnline() && visionCoprocessor.isCvOnline())
        {
            // Gets current chassis yaw angle
            float currentX = odometryInterface.getCurrentLocation2D().getX();
            float currentY = odometryInterface.getCurrentLocation2D().getY();
            float chassisYawAngle = odometryInterface.getYaw();

            const float maxWheelSpeed = HolonomicChassisSubsystem::getMaxWheelSpeed(
                drivers.refSerial.getRefSerialReceivingData(),
                drivers.refSerial.getRobotData().chassis.powerConsumptionLimit);

            Position setpoint = path.setInterpolatedPoint(currentPos);
            float x = 0.0;
            float y = 0.0;

            float desiredVelocityX = setpoint.x() - currentX;
            float desiredVelocityY = setpoint.y() - currentY;
            float mag = sqrtf(pow(desiredVelocityX, 2) + pow(desiredVelocityY, 2));
            if (mag > 0.01)
            {
                x = desiredVelocityX / mag * maxWheelSpeed;
                y = desiredVelocityY / mag * maxWheelSpeed;
            }

            x *= SPEED_FACTOR;
            y *= SPEED_FACTOR;
        }

        // Rotate X and Y depending on turret angle
        tap::algorithms::rotateVector(&x, &y, -chassisYawAngle); // @todo: we shouldn't need to negate this just for the sentry
        // we should debug in ozone to see which implementation is correct, and, if necessary
        // negate the yaw in the sentry kf odometry or negate the yaw in the chassiskf odometry
        // this negation has the potential to mess some things up!

        // set outputs
        chassis.setDesiredOutput(x, y, 0);

        lastX = currentX;
        lastY = currentY;
        desiredX = setpointData.x;
        desiredY = setpointData.y;
        }

    }
}

