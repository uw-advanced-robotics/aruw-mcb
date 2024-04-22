#include "auto_nav_path.hpp"
#include <deque>

void AutoNavPath::addPoint(AutoNavSetpointData point) {
    setpointData().push_back(point);
}

Position AutoNavPath::getSetPoint() {

}

float AutoNavPath::getDistanceToSegment(Position current, Position p1, Position p2) {
//     function distanceToSegment( v, a, b ) --v is the point, a and b are the start and end points of the line
//     local ab = b - a
//     local av = v - a

//     if (av:Dot(ab) <= 0) then -- Point is lagging behind start of the segment, so
//         return av.Magnitude -- Use distance to start of segment instead.
//     end

//     local bv = v - b

//     if (bv:Dot(ab) >= 0 ) then -- Point is advanced past the end of the segment, so
//         return bv.Magnitude -- Use distance to end of the segment instead
//     end

//     return (ab:Cross(av)).Magnitude / ab.Magnitude -- Perpendicular distance of point to segment
    float segmentLength = getDistance(p2 - p1);
    float distance1 = getDistance(p1 - current);
    if (dotprod(segmentLength, distance1) <= 0) {
        return distance1.magnitude;
    }
    float distance2 = getDistance(p2 - current);
    if (dotprod(segmentLength, distance2) > 0) {
        return distance2.magnitude;
    }

}

Position AutoNavPath::findInterpolatedPoint(Position closest) {
    float distanceRemaining = interpolationDistance;
    distanceRemaining -= getDistance(closest, setpointData[0]);
    for (int i = 0; i < setpointData.size()-1; i++) {
        float distance = getDistance(setpointData[i], setpointData[i+1]);
        if (distance <= distanceRemaining) {
            distanceRemaining -= distance;
        } else {
            float ratio = distanceRemaining / distance;
            float interpolatedX = setpointData[i].x + ratio * (setpointData[i+1].x - setpointData[i].x);
            float interpolatedY = setpointData[i].y + ratio * (setpointData[i+1].y - setpointData[i].y);
            return Position(interpolatedX, interpolatedY, 0);
        }
    }
    return Position(setpointData.back().x, setpointData.back().y, 0);
}

float AutoNavPath::getDistance(AutoNavSetpointData p1, AutoNavSetpointData p2) {
    return sqrt(p1.x*p1.x + p1.y*p1.y);
}




