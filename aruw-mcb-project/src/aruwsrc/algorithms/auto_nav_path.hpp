#ifndef AUTO_NAV_PATH_HPP_
#define AUTO_NAV_PATH_HPP_

#include <deque>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/transforms/position.hpp"

//#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

using namespace tap::algorithms::transforms;

namespace aruwsrc::algorithms
{
class AutoNavPath
{
public:
    AutoNavPath(float distance)
        : setpointData(),
          interpolationDistance(distance),
          oldSetpoint(0, 0, 0),
          currentSetpoint(0, 0, 0)
    { 
        setpointData.push_back(Position(0.1, 0.1, 0));
    }
    //void pushPoint(struct aruwsrc::serial::VisionCoprocessor::AutoNavSetpointData point);
    void pushPoint(Position point);
    void popPoint();
    void resetPath();
    bool empty() const { return setpointData.empty(); }
    Position getSetPoint() const;
    Position setInterpolatedPoint(Position current);
    Position pointFromParameter(const float parameter) const;

private:
    Position findClosestPoint(Position current);
    float getDistance(Position p1, Position p2) const;

    std::deque<Position> setpointData;

    const float interpolationDistance;  //
    Position oldSetpoint;               // The last setpoint used along the previous path
    Position currentSetpoint;

    bool path_interpolated = false;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
#endif
    Position getClosestOnSegment(Position current, Position p1, Position p2) const;
    float getClosestParameterOnSegment(Position current, Position p1, Position p2) const;
};

}  // namespace aruwsrc::algorithms

#endif  // AUTO_NAV_PATH_HPP_
