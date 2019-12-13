#include <math.h>
#include "contiguous_float.hpp"

namespace aruwlib
{

namespace algorithms
{

ContiguousFloat::ContiguousFloat(float value, float lowerBound, float upperBound)
{
    this->value = value;

    this->lowerBound = lowerBound;
    this->upperBound = upperBound;

    this->validateBounds();
    this->reboundValue();
}

ContiguousFloat::ContiguousFloat(float lowerBound, float upperBound) {
    this->lowerBound = lowerBound;
    this->upperBound = upperBound;
    this->validateBounds();
    this->value = 0;
}

ContiguousFloat::ContiguousFloat() {
    this->value = 0;
    this->lowerBound = 0;
    this->upperBound = 0;
}

float ContiguousFloat::reboundValue() {
    if (value < lowerBound) {
        value = upperBound
            + fmod(value - lowerBound, upperBound - lowerBound);
    } else if (value > upperBound) {
        value = lowerBound
            + fmod(value - upperBound, upperBound - lowerBound);
    }

    return value;
}

float ContiguousFloat::unwrapBelow() const {
    return lowerBound - (upperBound - value);
}

float ContiguousFloat::unwrapAbove() const {
    return upperBound + (value - lowerBound);
}

float ContiguousFloat::difference(const float otherValue) {
    return difference(ContiguousFloat(otherValue, lowerBound, upperBound));
}

float ContiguousFloat::difference(const ContiguousFloat otherValue) {
    // Find the shortest path to the target (smallest difference)
    float aboveDiff = otherValue.getValue() - this->unwrapAbove();
    float belowDiff = otherValue.getValue() - this->unwrapBelow();
    float stdDiff = otherValue.getValue() - this->getValue();

    float finalDiff = stdDiff;

    if (
        fabs(aboveDiff) < fabs(belowDiff)
        && fabs(aboveDiff) < fabs(stdDiff)
    ) {
        finalDiff = aboveDiff;
    }
    else if (
        fabs(belowDiff) < fabs(aboveDiff)
        && fabs(belowDiff) < fabs(stdDiff)
    ) {
        finalDiff = belowDiff;
    }

    return finalDiff;
}

void ContiguousFloat::shiftBounds(const float shiftMagnitude) {
    upperBound += shiftMagnitude;
    lowerBound += shiftMagnitude;
    reboundValue();
}

void ContiguousFloat::shiftValue(const float shiftMagnitude) {
    value += shiftMagnitude;
    reboundValue();
}

// Getters/Setters ----------------
// Value
float ContiguousFloat::getValue() const {
    return value;
}

void ContiguousFloat::setValue(const float newValue) {
    value = newValue;
    this->reboundValue();
}

// Upper bound
float ContiguousFloat::getUpperBound() const {
    return upperBound;
}

void ContiguousFloat::setUpperBound(const float newValue) {
    upperBound = newValue;

    this->validateBounds();
    this->reboundValue();
}

// Lower bound
float ContiguousFloat::getLowerBound() const {
    return lowerBound;
}

void ContiguousFloat::setLowerBound(const float newValue) {
    lowerBound = newValue;

    this->validateBounds();
    this->reboundValue();
}

void ContiguousFloat::validateBounds() {
    if (lowerBound > upperBound) {
        float tmp = this->lowerBound;
        this->lowerBound = this->upperBound;
        this->upperBound = tmp;
    }
}

}  //  namespace aruwlib

}  // namespace algorithms
