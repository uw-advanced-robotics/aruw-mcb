#include "FloatProperty.hpp"

namespace aruwlib
{

bool operator==(FloatProperty p1, FloatProperty p2)
{
    return p1.data == p2.data;
}

bool operator!=(FloatProperty p1, FloatProperty p2)
{
    return p1.data != p2.data;
}

bool operator<(FloatProperty p1, FloatProperty p2)
{
    return p1.data < p2.data;
}

bool operator>(FloatProperty p1, FloatProperty p2)
{
    return p1.data > p2.data;
}

bool operator<=(FloatProperty p1, FloatProperty p2)
{
    return !(p1 > p2);
}

bool operator>=(FloatProperty p1, FloatProperty p2)
{
    return !(p1 < p2);
}

FloatProperty operator+(FloatProperty p1, FloatProperty p2)
{
    return FloatProperty(p1.data + p2.data);
}

FloatProperty operator+(FloatProperty p1, float p2)
{
    return FloatProperty(p1.data + p2);
}

FloatProperty operator+(float p1, FloatProperty p2)
{
    return FloatProperty(p1 + p2.data);
}

FloatProperty FloatProperty::operator=(FloatProperty other)
{
    if (this != &other)
    {
        data = other.data;
    }
    return *this;
}

FloatProperty FloatProperty::operator=(float other)
{
    data = other;
    return *this;
}

FloatProperty FloatProperty::operator+=(FloatProperty other)
{
    data += other.data;
    return *this;
}

FloatProperty FloatProperty::operator+=(float other)
{
    data += other;
    return *this;
}

FloatProperty FloatProperty::operator-=(FloatProperty other)
{
    data -= other.data;
    return *this;
}

FloatProperty FloatProperty::operator-=(float other)
{
    data -= other;
    return *this;
}

FloatProperty FloatProperty::operator*=(FloatProperty other)
{
    data *= other.data;
    return *this;
}

FloatProperty FloatProperty::operator*=(float other)
{
    data *= other;
    return *this;
}

FloatProperty FloatProperty::operator/=(FloatProperty other)
{
    data /= other.data;
    return *this;
}

FloatProperty FloatProperty::operator/=(float other)
{
    data /= other;
    return *this;
}


} // namespace aruwlib
