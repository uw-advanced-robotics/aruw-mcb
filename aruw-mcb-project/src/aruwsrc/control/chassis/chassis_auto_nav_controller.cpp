#include "chassis_auto_nav_controller.hpp"

namespace aruwsrc::chassis
{
    void ChassisAutoNavController::runController(const uint32_t dt, Position currentPos) {
        // get current position
        Position setPoint = path.setInterpolatedPoint(currentPos);
        //TODO: ...
    }
}