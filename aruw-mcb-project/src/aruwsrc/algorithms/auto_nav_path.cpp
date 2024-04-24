#include "auto_nav_path.hpp"
#include <deque>

void AutoNavPath::pushPoint(AutoNavSetpointData point) {
    setpointData.push_back(point);
}

Position AutoNavPath::getSetPoint() const {
    return Position(currentSetpoint.x, currentSetpoint.y, 0);
}

Position AutoNavPath::getClosestOnSegment(Position current, Position p1, Position p2) {
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
        return p1 + Position(distance1x * ratio, distance1y * ratio, 0);
    }
}

Position AutoNavPath::findInterpolatedPoint(Position closest) const {
    float distanceRemaining = interpolationDistance;
    distanceRemaining -= getDistance(closest, Position(setpointData[0].x, setpointData[0].y, 0));
    for (size_t i = 0; i < setpointData.size()-1; i++) {
        Position p1 = Position(setpointData[i].x, setpointData[i].y, 0);
        Position p2 = Position(setpointData[i+1].x, setpointData[i+1].y, 0);
        float distance = getDistance(p1, p2);
        if (distance <= distanceRemaining) {
            distanceRemaining -= distance;
        } else {
            float ratio = distanceRemaining / distance;
            float interpolatedX = setpointData[i].x + ratio * (p2.x() - p1.x());
            float interpolatedY = setpointData[i].y + ratio * (p2.y() - p1.y());
            return Position(interpolatedX, interpolatedY, 0);
        }
    }
    return Position(setpointData.back().x, setpointData.back().y, 0);
}

float AutoNavPath::getDistance(Position p1, Position p2) const {
    float distanceX = p2.x() - p1.x();
    float distanceY = p2.y() - p1.y();
    return sqrt(distanceX * distanceX + distanceY * distanceY);
}
