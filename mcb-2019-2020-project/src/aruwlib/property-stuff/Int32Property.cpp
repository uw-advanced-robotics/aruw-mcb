#include "Int32Property.hpp"

namespace aruwlib
{

bool operator==(Int32Property p1, Int32Property p2)
{
    return p1.data == p2.data;
}

bool operator!=(Int32Property p1, Int32Property p2)
{
    return p1.data != p2.data;
}

bool operator<(Int32Property p1, Int32Property p2)
{
    return p1.data < p2.data;
}

bool operator>(Int32Property p1, Int32Property p2)
{
    return p1.data > p2.data;
}

bool operator<=(Int32Property p1, Int32Property p2)
{
    return !(p1 > p2);
}

bool operator>=(Int32Property p1, Int32Property p2)
{
    return !(p1 < p2);
}

Int32Property operator+(Int32Property p1, Int32Property p2)
{
    return Int32Property(p1.data + p2.data);
}

Int32Property operator+(Int32Property p1, int32_t p2)
{
    return Int32Property(p1.data + p2);
}

Int32Property operator+(int32_t p1, Int32Property p2)
{
    return Int32Property(p1 + p2.data);
}

Int32Property Int32Property::operator=(Int32Property other)
{
    if (this != &other)
    {
        data = other.data;
    }
    return *this;
}
Int32Property Int32Property::operator=(int32_t other)
{
    data = other;
    return *this;
}

Int32Property Int32Property::operator+=(Int32Property other)
{
    data += other.data;
    return *this;
}

Int32Property Int32Property::operator+=(int32_t other)
{
    data += other;
    return *this;
}

Int32Property Int32Property::operator-=(Int32Property other)
{
    data -= other.data;
    return *this;
}

Int32Property Int32Property::operator-=(int32_t other)
{
    data -= other;
    return *this;
}

Int32Property Int32Property::operator*=(Int32Property other)
{
    data *= other.data;
    return *this;
}

Int32Property Int32Property::operator*=(int32_t other)
{
    data *= other;
    return *this;
}

Int32Property Int32Property::operator/=(Int32Property other)
{
    data /= other.data;
    return *this;
}

Int32Property Int32Property::operator/=(int32_t other)
{
    data /= other;
    return *this;
}

} // namespace aruwlib
