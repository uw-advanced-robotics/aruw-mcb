#include "Distance.hpp"

#include "math_user_utils.hpp"

namespace aruwlib
{
namespace algorithms
{

Distance::Distance() : distance(0.0f), unit(CM) {}

Distance::Distance(float distance, Units unit) : distance(distance), unit(unit) {}

void Distance::set(float val) { distance = val; }

float Distance::asMm() const
{
    switch (unit)
    {
        case MM: return distance;
        case CM: return distance * MM_PER_CM;
        case M: return distance * MM_PER_CM * CM_PER_M;
        default: return 0.0f;
    }
}

float Distance::asCm() const
{
    switch(unit)
    {
        case MM: return distance / MM_PER_CM;
        case CM: return distance;
        case M: return distance * CM_PER_M;
        default: return 0.0f;
    }
}

float Distance::asM() const
{
    switch(unit)
    {
        case MM: return distance / MM_PER_CM / CM_PER_M;
        case CM: return distance / CM_PER_M;
        case M: return distance;
        default: return 0.0f;
    }
}

Distance Distance::toMm() const
{
    return Distance(asMm(), MM);
}

Distance Distance::toCm() const
{
    return Distance(asCm(), CM);
}

Distance Distance::toM() const
{
    return Distance(asM(), M);
}


Distance Distance::operator+(Distance other) const
{
    float distM = asM();
    float otherDistM = other.asM();
    Distance sum(distM + otherDistM, M);
    switch(unit)
    {
        case MM: return sum.toMm();
        case CM: return sum.toCm();
        case M: return sum.toM();
        default: return sum;
    }
    // return Distance(distM + other)
}

Distance Distance::operator-(Distance other) const
{
    other.distance = -other.distance;
    return *this + other;
}

Distance Distance::operator*(float val) const
{
    return Distance(distance * val, unit);
}

Distance Distance::operator/(float val) const
{
    return Distance(distance / val, unit);
}

Distance Distance::operator+=(Distance other)
{
    *this = *this + other;
}

Distance Distance::operator-=(Distance other)
{
    *this = *this - other;
}

Distance Distance::operator*=(float val)
{
    *this = *this * val;
}

Distance Distance::operator/=(float val)
{
    *this = *this / val;
}

bool Distance::operator<(Distance other) const
{
    return asM() - other.asM() < 0.0f;
}

bool Distance::operator>(Distance other) const
{
    return asM() - other.asM() > 0.0f;
}

bool Distance::operator<=(Distance other) const
{
    return !(*this > other);
}

bool Distance::operator>=(Distance other) const
{
    return !(*this < other);
}

bool Distance::operator==(Distance other) const
{
    return compareFloatClose(asM(), other.asM(), EQUAL_EPSILON);
}

bool Distance::operator!=(Distance other) const
{
    return !(*this == other);
}

}  // namespace algorithms
}  // namespace aruwlib
