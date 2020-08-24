#include "Distance.hpp"

#include "math_user_utils.hpp"

namespace aruwlib
{
namespace algorithms
{
Distance::Distance() : distance(0.0f), unit(Units::CM) {}

Distance::Distance(float distance, Units unit) : distance(distance), unit(unit) {}

void Distance::set(float val) { distance = val; }

float Distance::asMm() const
{
    switch (unit)
    {
        case Units::MM:
            return distance;
        case Units::CM:
            return distance * MM_PER_CM;
        case Units::M:
            return distance * MM_PER_CM * CM_PER_M;
        default:
            return 0.0f;
    }
}

float Distance::asCm() const
{
    switch (unit)
    {
        case Units::MM:
            return distance / MM_PER_CM;
        case Units::CM:
            return distance;
        case Units::M:
            return distance * CM_PER_M;
        default:
            return 0.0f;
    }
}

float Distance::asM() const
{
    switch (unit)
    {
        case Units::MM:
            return distance / MM_PER_CM / CM_PER_M;
        case Units::CM:
            return distance / CM_PER_M;
        case Units::M:
            return distance;
        default:
            return 0.0f;
    }
}

Distance Distance::toMm() const { return Distance(asMm(), Units::MM); }

Distance Distance::toCm() const { return Distance(asCm(), Units::CM); }

Distance Distance::toM() const { return Distance(asM(), Units::M); }

Distance Distance::operator+(Distance other) const
{
    float distM = asM();
    float otherDistM = other.asM();
    Distance sum(distM + otherDistM, Units::M);
    switch (unit)
    {
        case Units::MM:
            return sum.toMm();
        case Units::CM:
            return sum.toCm();
        case Units::M:
            return sum.toM();
        default:
            return sum;
    }
    // return Distance(distM + other)
}

Distance Distance::operator-(Distance other) const
{
    other.distance = -other.distance;
    return *this + other;
}

Distance Distance::operator*(float val) const { return Distance(distance * val, unit); }

Distance Distance::operator*(Distance other) const { return *this * other.distance; }

Distance Distance::operator/(float val) const { return Distance(distance / val, unit); }

Distance Distance::operator/(Distance other) const { return *this / other.distance; }

Distance Distance::operator+=(Distance other)
{
    *this = *this + other;
    return *this;
}

Distance Distance::operator-=(Distance other)
{
    *this = *this - other;
    return *this;
}

Distance Distance::operator*=(float val)
{
    *this = *this * val;
    return *this;
}

Distance Distance::operator*=(Distance other)
{
    *this = *this * other;
    return *this;
}

Distance Distance::operator/=(float val)
{
    *this = *this / val;
    return *this;
}

Distance Distance::operator/=(Distance other)
{
    *this = *this / other;
    return *this;
}

Distance Distance::operator++()
{
    this->distance++;
    return *this;
}

Distance Distance::operator--()
{
    this->distance--;
    return *this;
}

bool Distance::operator<(Distance other) const { return asM() < other.asM(); }

bool Distance::operator>(Distance other) const { return asM() > other.asM(); }

bool Distance::operator<=(Distance other) const { return !(*this > other); }

bool Distance::operator>=(Distance other) const { return !(*this < other); }

bool Distance::operator==(Distance other) const
{
    return compareFloatClose(asM(), other.asM(), EQUAL_EPSILON);
}

bool Distance::operator!=(Distance other) const { return !(*this == other); }

}  // namespace algorithms
}  // namespace aruwlib
