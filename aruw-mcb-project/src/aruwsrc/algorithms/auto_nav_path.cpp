#include "auto_nav_path.hpp"
#include <deque>

#include "tap\algorithms\transforms\vector.hpp"
#include "tap\algorithms\transforms\position.hpp"

#include "aruwsrc\communication\serial\vision_coprocessor.hpp"

using tap::algorithms::transforms::Position;

void AutoNavPath::pushPoint(aruwsrc::serial::VisionCoprocessor::AutoNavSetpointData point) {
    setpointData.push_back(Position(point.x, point.y, 0));
}

void AutoNavPath::pushPoint(Position point) {
    setpointData.push_back(point);
}

void AutoNavPath::popPoint() {
    setpointData.pop_front();
}

void AutoNavPath::resetPath() {
    setpointData.clear();
    oldSetpoint = currentSetpoint;
}

Position AutoNavPath::getSetPoint() const {
    return currentSetpoint;
}

Position AutoNavPath::getClosestOnSegment(Position current, Position p1, Position p2) const {
    float distance1x = p2.x() - p1.x();
    float distance1y = p2.y() - p1.y();
    float distance2x = current.x() - p1.x();
    float distance2y = current.y() - p1.y();
    float dotprod = distance1x * distance2x + distance1y * distance2y;
    float ratio = dotprod / (distance1x * distance1x + distance1y * distance1y);
    if (ratio < 0) {
        return p1;
    } else if (ratio > 1) {
        return p2;
    } else {
        return Position(p1.x() + distance1x * ratio, p1.y() + distance1y * ratio, 0);
    }
}

Position AutoNavPath::findClosestPoint(Position current) {
    float minDistance = F32_MAX;
    size_t closestIndex = 0;
    Position minClosest = Position(0,0,0);
    for (size_t i = 0; i < setpointData.size() - 1; i++) {
        Position p1 = setpointData[i];
        Position p2 = setpointData[i+1];
        Position closest = getClosestOnSegment(current, p1, p2);
        float distance = getDistance(current, closest);
        if (distance < minDistance) {
            minDistance = distance;
            minClosest = closest;
            closestIndex = i;
        }
    }

    // remove points behind where we currently are
    for (size_t i = 0; i < closestIndex; i++) {
        popPoint();
    }
    return minClosest;
}

Position AutoNavPath::setInterpolatedPoint(Position current) {
    if (setpointData.empty()) {
        return current;
    }
    Position closest = findClosestPoint(current);
    float offset = getDistance(setpointData[0], closest);
    float offsetDistance = offset + interpolationDistance;
    size_t i = (size_t)offsetDistance; // floor of distance from first path point

    if (i >= setpointData.size() - 1) {
        return setpointData.back();
    }

    Position p1 = setpointData[i];
    Position p2 = setpointData[i+1];
    
    float ratio = offsetDistance - i;
    float interpolatedX = p1.x() + ratio * (p2.x() - p1.x());
    float interpolatedY = p1.y() + ratio * (p2.y() - p1.y());

    currentSetpoint = Position(interpolatedX, interpolatedY, 0);

    return currentSetpoint;
}

float AutoNavPath::getDistance(Position p1, Position p2) const {
    float distanceX = p2.x() - p1.x();
    float distanceY = p2.y() - p1.y();
    return sqrt(distanceX * distanceX + distanceY * distanceY);
}
