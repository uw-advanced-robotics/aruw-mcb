#include "auto_nav_path.hpp"

#include <deque>

#include "tap/algorithms/transforms/position.hpp"
#include "tap/algorithms/transforms/vector.hpp"

using tap::algorithms::transforms::Position;
using namespace aruwsrc::algorithms;

void AutoNavPath::pushPoint(Position point) { setpointData.push_back(point); }

void AutoNavPath::pushFront(Position point) { setpointData.push_front(point); }

void AutoNavPath::popPoint() { setpointData.pop_front(); }

void AutoNavPath::resetPath()
{
    setpointData.clear();
    path_changed = true;
}

bool AutoNavPath::hasChanged() const { return path_changed; }
void AutoNavPath::togglePathChanged() { path_changed = !path_changed; }

float AutoNavPath::positionToClosestParameter(const Position pos) const
{
    float minDistance = F32_MAX;
    float minClosest = 0.0f;
    float currParameter = 0.0f;

    if (setpointData.size() < 2)
    {
        return 0.0f;
    }

    for (size_t i = 0; i < setpointData.size() - 1; i++)
    {
        Position p1 = setpointData[i];
        Position p2 = setpointData[i + 1];

        float distance = Position::distance(p1, p2);  // segment distance
        float paramOnSegment =
            getClosestParameterOnSegment(pos, p1, p2);  // parameter along segment of closest point
        Position closestPoint = Position::interpolate(
            p1,
            p2,
            paramOnSegment / distance);                  // position of closest point on segment
        float closest = currParameter + paramOnSegment;  // total parameter of closest point on path

        float distanceToClosest = Position::distance(pos, closestPoint);

        if (distanceToClosest < minDistance)
        {
            minDistance = distanceToClosest;
            minClosest = closest;
        }
        currParameter += distance;
    }
    return minClosest;
}

Position AutoNavPath::parametertoPosition(const float parameter) const
{
    // assert(!setpointData.empty());

    int pointIndex;
    float currParameter = 0;
    float segmentDistance = 0;
    for (pointIndex = 0; pointIndex < (int)setpointData.size() - 1; pointIndex++)
    {
        segmentDistance =
            Position::distance(setpointData[pointIndex], setpointData[pointIndex + 1]);
        if (currParameter + segmentDistance > parameter)
        {
            break;
        }
        currParameter += segmentDistance;
    }

    if ((size_t)pointIndex + 2 > setpointData.size())
    {
        return setpointData.back();
    }

    return Position::interpolate(
        setpointData[pointIndex],
        setpointData[pointIndex + 1],
        (parameter - currParameter) / segmentDistance);
}

// float AutoNavPath::parameterToSpeed(const float parameter) const
// {
//     // TODO: IMPLEMENT THIS!!
//     // currently just test value
//     return 0.8;
// }

float AutoNavPath::getClosestParameterOnSegment(Position current, Position p1, Position p2) const
{
    Vector distance1 = p2 - p1;
    Vector distance2 = current - p1;
    float dotprod = distance1.dot(distance2);
    float ratio = dotprod / (distance1.dot(distance1));
    return tap::algorithms::limitVal(ratio, 0.0f, 1.0f) * Position::distance(p1, p2);
}
