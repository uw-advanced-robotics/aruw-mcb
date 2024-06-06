/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "auto_nav_path.hpp"

#include <deque>

#include "tap/algorithms/transforms/position.hpp"
#include "tap/algorithms/transforms/vector.hpp"

using tap::algorithms::transforms::Position;
using namespace aruwsrc::algorithms;

void AutoNavPath::pushPoint(Position point) { setpointData.push_back(point); }

void AutoNavPath::popPoint() { setpointData.pop_front(); }

void AutoNavPath::resetPath()
{
    setpointData.clear();
    pathChanged = true;
}

bool AutoNavPath::hasChanged() const { return pathChanged; }

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
    size_t pointIndex;
    float currParameter = 0, segmentDistance = 0;
    for (pointIndex = 0; pointIndex < setpointData.size() - 1; pointIndex++)
    {
        segmentDistance =
            Position::distance(setpointData[pointIndex], setpointData[pointIndex + 1]);
        if (currParameter + segmentDistance > parameter)
        {
            break;
        }
        currParameter += segmentDistance;
    }

    if (pointIndex + 2 > setpointData.size())
    {
        return setpointData.back();
    }

    return Position::interpolate(
        setpointData[pointIndex],
        setpointData[pointIndex + 1],
        (parameter - currParameter) / segmentDistance);
}

float AutoNavPath::getClosestParameterOnSegment(Position current, Position p1, Position p2) const
{
    Vector distance1 = p2 - p1;
    Vector distance2 = current - p1;
    float dotprod = distance1.dot(distance2);
    float ratio = dotprod / (distance1.dot(distance1));
    return tap::algorithms::limitVal(ratio, 0.0f, 1.0f) * Position::distance(p1, p2);
}
