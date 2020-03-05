#ifndef __LINEAR_INTERPOLATION_HPP__
#define __LINEAR_INTERPOLATION_HPP__

#include <rm-dev-board-a/board.hpp>

/**
 * Solving the linear equation Au = x, for u then plugging u into
 * y(t) = u_0 + u_1 * t + u_2 * t^2
 * When frequency is assumed to be constant, u can be calculated fast.
 */
class LinearInterpolation
{
 public:
    LinearInterpolation(float frequency);

    void update(float newY);

    // use modm::Clock::now().getTime()
    float getInterpolatedValue(uint32_t currTime);

 private:
    int currInitialPoint ;
    float frequency;
    float yValues[3];
    uint32_t lastUpdateCallTime;
    float prevY;
    float slope;

    float getY1();

    float getY2();

    float getY3();
};

#endif
