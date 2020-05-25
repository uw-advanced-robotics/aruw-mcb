#include "FloatProperty.hpp"

namespace aruwlib
{

uint8_t *FloatProperty::serializeData(uint16_t *size) const
{
    *size = sizeof(float);
    uint8_t *arr = new uint8_t[*size];
    memcpy(arr, &data, *size);
    return arr;
}

FloatProperty& FloatProperty::operator=(float other)
{
    data = other;
    return *this;
}

FloatProperty& FloatProperty::operator+=(FloatProperty& other)
{
    data += other.data;
    return *this;
}

FloatProperty& FloatProperty::operator+=(float other)
{
    data += other;
    return *this;
}

FloatProperty& FloatProperty::operator-=(FloatProperty& other)
{
    data -= other.data;
    return *this;
}

FloatProperty& FloatProperty::operator-=(float other)
{
    data -= other;
    return *this;
}

FloatProperty& FloatProperty::operator*=(FloatProperty& other)
{
    data *= other.data;
    return *this;
}

FloatProperty& FloatProperty::operator*=(float other)
{
    data *= other;
    return *this;
}

FloatProperty& FloatProperty::operator/=(FloatProperty& other)
{
    data /= other.data;
    return *this;
}

FloatProperty& FloatProperty::operator/=(float other)
{
    data /= other;
    return *this;
}


} // namespace aruwlib
