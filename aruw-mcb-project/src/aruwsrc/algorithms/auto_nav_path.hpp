#ifndef AUTO_NAV_PATH_HPP_
#define AUTO_NAV_PATH_HPP_

#include <deque>
#include "tap\algorithms\transforms\position.hpp"
#include "aruwsrc\communication\serial\vision_coprocessor.hpp"


using namespace tap::algorithms::transforms;

class AutoNavPath {
public:
    AutoNavPath(float distance):
        setpointData(),
        interpolationDistance(distance),
        oldSetpoint(0,0,0),
        currentSetpoint(0,0,0),
        nextPathPoint(0,0,0) {}
    void pushPoint(aruwsrc::serial::VisionCoprocessor::AutoNavSetpointData point);
    void pushPoint(Position point);
    void popPoint();
    Position getSetPoint() const;
    Position setInterpolatedPoint(Position current);

private:
    Position findClosestPoint(Position current);
    // TODO: should this be placed in Vector as magnitude()??... Should we even be using transforms library functions?!?!?!?!
    float getDistance(Position p1, Position p2) const; 

    std::deque<Position> setpointData;

    float interpolationDistance;
    // The last setpoint used along the previous path
    Position oldSetpoint;
    Position currentSetpoint;
    Position nextPathPoint;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
#endif
    Position getClosestOnSegment(Position current, Position p1, Position p2) const;
};

#endif // AUTO_NAV_PATH_HPP_
