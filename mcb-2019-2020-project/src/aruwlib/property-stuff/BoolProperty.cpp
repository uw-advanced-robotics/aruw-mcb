#include "BoolProperty.hpp"

namespace aruwlib
{

bool operator==(BoolProperty p1, BoolProperty p2)
{
    return p1.data == p2.data;
}

bool operator==(bool p1, BoolProperty p2)
{
    return p1 == p2.data;
}

bool operator==(BoolProperty p1, bool p2)
{
    return p1.data == p2;
}

bool operator!=(BoolProperty p1, BoolProperty p2)
{
    return p1.data != p2.data;
}

bool operator!=(BoolProperty p1, bool p2)
{
    return p1.data != p2;
}

bool operator!=(bool p1, BoolProperty p2)
{
    return p1 != p2.data;
}

bool operator||(BoolProperty p1, BoolProperty p2)
{
    return p1.data || p2.data;
}

bool operator||(BoolProperty p1, bool p2)
{
    return p1.data || p2;
}

bool operator||(bool p1, BoolProperty p2)
{
    return p1 || p2.data;
}

bool operator&&(BoolProperty p1, BoolProperty p2)
{
    return p1.data && p2.data;
}

bool operator&&(BoolProperty p1, bool p2)
{
    return p1.data && p2;
}

bool operator&&(bool p1, BoolProperty p2)
{
    return p1 && p2.data;
}

BoolProperty BoolProperty::operator=(BoolProperty p2)
{
    if (this != &p2)
    {
        data = p2.data;
    }
    return *this;
}

BoolProperty BoolProperty::operator=(bool p2)
{
    data = p2;
    return *this;
}

BoolProperty BoolProperty::operator&=(BoolProperty p1)
{
    data &= p1.data;
    return *this;
}

BoolProperty BoolProperty::operator&=(bool p1)
{
    data &= p1;
    return *this;
}

BoolProperty BoolProperty::operator|=(BoolProperty p1)
{
    data |= p1.data;
    return *this;
}

BoolProperty BoolProperty::operator|=(bool p1)
{
    data |= p1;
    return *this;
}

};  // namespace aruwlib
