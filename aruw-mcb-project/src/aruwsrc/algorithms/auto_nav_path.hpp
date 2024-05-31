#ifndef AUTO_NAV_PATH_HPP_
#define AUTO_NAV_PATH_HPP_

#include <deque>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/transforms/position.hpp"

// #include "aruwsrc/communication/serial/vision_coprocessor.hpp"

using namespace tap::algorithms::transforms;

namespace aruwsrc::algorithms
{
class AutoNavPath
{
public:
    AutoNavPath() : setpointData(), path_changed(false) {}

    void pushPoint(Position point);
    void pushFront(Position point);
    void popPoint();
    void resetPath();
    bool empty() const { return setpointData.empty(); }
    float positionToClosestParameter(const Position pos) const;
    Position parametertoPosition(const float parameter) const;
    float parameterToSpeed(const float parameter) const;
    bool hasChanged() const;
    void togglePathChanged();

    inline float distTo(const Position& position, const float parameter) const
    {
        return Position::distance(position, parametertoPosition(parameter));
    }

private:
    std::deque<Position> setpointData;
    bool path_changed;

    bool path_interpolated = false;  // DEBUG

    float getDistance(const Position p1, const Position p2) const;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
#endif
    float getClosestParameterOnSegment(Position current, Position p1, Position p2) const;
};

}  // namespace aruwsrc::algorithms

#endif  // AUTO_NAV_PATH_HPP_
