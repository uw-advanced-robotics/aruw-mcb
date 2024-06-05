#ifndef AUTO_NAV_PATH_HPP_
#define AUTO_NAV_PATH_HPP_

#include <deque>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/transforms/position.hpp"

using namespace tap::algorithms::transforms;

namespace aruwsrc::algorithms
{
/**
 * A linearly interpolated set points used for autonomous navigation. It's essentially a 2d
 * parametric function of arc length. It is interacted with via a path 'parameter', which is a
 * point's distance along the path.
 */
class AutoNavPath
{
public:
    AutoNavPath() : setpointData(), path_changed(false) {}

    /**
     * Appends the given point to the path.
     */
    void pushPoint(Position point);

    /**
     * Removes the first point from the path.
     */
    void popPoint();

    /**
     * Clears all points in the path.
     */
    void resetPath();

    /**
     * Returns whether the path doesn't have any points.
     */
    bool empty() const { return setpointData.empty(); }

    /**
     * Finds the closest position along the path to the given position and returns that position's
     * arc length parameterized value (its distance along the path).
     */
    float positionToClosestParameter(const Position pos) const;

    /**
     * Returns the point the given distance along the path.
     */
    Position parametertoPosition(const float parameter) const;

    /**
     * Returns whether the path was updated.
     */
    bool hasChanged() const;

    void togglePathChanged();

    inline float distTo(const Position& position, const float parameter) const
    {
        return Position::distance(position, parametertoPosition(parameter));
    }

private:
    std::deque<Position> setpointData;
    bool path_changed;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
#endif
    float getClosestParameterOnSegment(Position current, Position p1, Position p2) const;
};

}  // namespace aruwsrc::algorithms

#endif  // AUTO_NAV_PATH_HPP_
