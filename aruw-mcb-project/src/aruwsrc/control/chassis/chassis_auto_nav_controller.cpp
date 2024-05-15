#include "chassis_auto_nav_controller.hpp"

namespace aruwsrc::chassis
{
    void ChassisAutoNavController::runController(const uint32_t dt, Position currentPos) {
        // get current position
        Position desiredSetPoint = path.setInterpolatedPoint(currentPos);
        //TODO: verify this is testable, test it, then replace with PID version
        float desiredVelocityX = desiredSetPoint.x() - currentPos.x();
        float desiredVelocityY = desiredSetPoint.y() - currentPos.y();
        float mag = sqrtf(pow(desiredVelocityX, 2) + pow(desiredVelocityY, 2));
        float WHEEL_SPEED_CONSTANT = 100.0f;
        chassis.setDesiredOutput(desiredVelocityX/mag * WHEEL_SPEED_CONSTANT, desiredVelocityY/mag * WHEEL_SPEED_CONSTANT, 0.0f);
    }
}