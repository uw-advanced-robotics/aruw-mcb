#include "auto_nav_path.hpp"

#include <deque>

#include "tap/algorithms/transforms/position.hpp"
#include "tap/algorithms/transforms/vector.hpp"

using tap::algorithms::transforms::Position;
using namespace aruwsrc::algorithms;

void AutoNavPath::pushPoint(Position point) { setpointData.push_back(point); }

void AutoNavPath::pushFront(Position point) { setpointData.push_front(point); }

void AutoNavPath::popPoint() { setpointData.pop_front(); }

void AutoNavPath::resetPath() { 
    setpointData.clear(); 
    path_changed = true;
}

bool AutoNavPath::hasChanged() const { return path_changed; }
void AutoNavPath::togglePathChanged() { path_changed = !path_changed; }

float debugMinClosest = 0;
float AutoNavPath::positionToClosestParameter(const Position pos) const
{
    float minDistance = F32_MAX;
    float minClosest = 0.0f;
    debugMinClosest = minClosest;

    for (size_t i = 0; i < setpointData.size() - 1; i++)
    {
        Position p1 = setpointData[i];
        Position p2 = setpointData[i + 1];
        float closest = i + getClosestParameterOnSegment(pos, p1, p2);
        float distance = distTo(pos, closest);
        if (distance < minDistance)
        {
            minDistance = distance;
            minClosest = closest;
        }
    }
    return minClosest;
}
int minPointIndex = 1000;
Position back(-1.0,-1.0,0.0);
Position firstPoint(-1.0,-1.0,0.0);
int dequeSize = -1;
float debugParameter = 1000.0;
Position AutoNavPath::parametertoPosition(const float parameter) const
{
    // assert(!setpointData.empty());
    debugParameter = parameter;
    int pointIndex = static_cast<int>(parameter);  // only works bc parameterized length every segment is
                                      // currently considered to be 1
    dequeSize = setpointData.size();
    minPointIndex = (pointIndex < minPointIndex) ? pointIndex : minPointIndex;

    if ((size_t) pointIndex + 2 > setpointData.size()) {
        back = setpointData.back();
        return back;
    }
    firstPoint = setpointData[pointIndex];

    return Position::interpolate(
        setpointData[pointIndex],
        setpointData[pointIndex + 1],
        parameter - pointIndex);
}

float AutoNavPath::parameterToSpeed(const float parameter) const
{
    // IMPLEMENT THIS!!
    // currently making warnings go away
    return parameter;
}

float AutoNavPath::getClosestParameterOnSegment(Position current, Position p1, Position p2) const
{
    float distance1x = p2.x() - p1.x();
    float distance1y = p2.y() - p1.y();
    float distance2x = current.x() - p1.x();
    float distance2y = current.y() - p1.y();
    float dotprod = distance1x * distance2x + distance1y * distance2y;
    float ratio = dotprod / (distance1x * distance1x + distance1y * distance1y);
    return tap::algorithms::limitVal(ratio, 0.0f, 1.0f);
}
