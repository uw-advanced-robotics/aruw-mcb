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
        interpolationDistance(distance) {}
    void addPoint(AutoNavSetpointData point);
    Position getSetPoint();

private:
    Position findClosestPoint(Position current);
    Position findInterpolatedPoint(Position closest);
    float getDistanceToSegment(Position current, Position p1, Position p2);
    float getDistance(AutoNavSetpointData p1, AutoNavSetpointData p2);

    std::deque<AutoNavSetpointData> setpointData();

    float interpolationDistance;
    // The last setpoint used along the previous path
    AutoNavSetpointData oldSetpoint;
    AutoNavSetpointData currentSetpoint;
};

#endif // AUTO_NAV_PATH_HPP_
