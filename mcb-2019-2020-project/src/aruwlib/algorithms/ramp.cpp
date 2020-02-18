#include <math.h>
#include "ramp.hpp"

namespace aruwlib
{

namespace algorithms
{

Ramp::Ramp(const float& initialValue) :
    target(initialValue),
    value(initialValue),
    targetReached(true)
{}

void Ramp::setTarget(const float& target)
{
    this->target = target;
    this->targetReached = false;
}

void Ramp::update(float increment)
{
    increment = fabs(increment);
    if (target > value)
    {
        float targetValueDifference = target - value;
        if (targetValueDifference > increment) {
            value += increment;
        }
        else
        {
            value = target;
            targetReached = true;
        }
    }
    else
    {
        float targetValueDifference = value - target;
        if (targetValueDifference > increment) {
            value -= increment;
        }
        else
        {
            value = target;
            targetReached = true;
        }
    }
}

void Ramp::reset(const float& value)
{
    this->value = value;
}

const float& Ramp::getValue() const
{
    return this->value;
}

bool Ramp::isTargetReached() const
{
    return targetReached;
}

}  // namespace algorithms

}  // namespace aruwlib
