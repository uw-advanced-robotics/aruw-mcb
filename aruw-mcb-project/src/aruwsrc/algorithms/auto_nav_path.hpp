#ifndef AUTO_NAV_PATH_HPP_
#define AUTO_NAV_PATH_HPP_

#include <deque>
#include "tap\algorithms\transforms\position.hpp"

using namespace tap::algorithms::transforms;

struct AutoNavSetpointData
{
    bool pathFound;
    float x;
    float y;
    unsigned long long timestamp;
};

class AutoNavPath {
public:
    AutoNavPath(float distance):
        setpointData(),
        interpolationDistance(distance) {}
    void pushPoint(AutoNavSetpointData point);
    void popPoint();
    Position getSetPoint() const;

private:
    Position findClosestPoint(Position current) const;
    Position findInterpolatedPoint(Position closest) const;
    // TODO: should this be placed in Vector as magnitude()??... Should we even be using transforms library functions?!?!?!?!
    float getDistance(Position p1, Position p2) const; 

    std::deque<AutoNavSetpointData> setpointData;

    float interpolationDistance;
    // The last setpoint used along the previous path
    AutoNavSetpointData oldSetpoint;
    AutoNavSetpointData currentSetpoint;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
#endif
    Position getClosestOnSegment(Position current, Position p1, Position p2);
};

#endif // AUTO_NAV_PATH_HPP_
