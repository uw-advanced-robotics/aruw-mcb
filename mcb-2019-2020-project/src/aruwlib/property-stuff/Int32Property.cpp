#include "Int32Property.hpp"

namespace aruwlib
{

uint8_t *Int32Property::serializeData(uint16_t *size) const
{
    *size = sizeof(int32_t);
    uint8_t *arr = new uint8_t[*size];
    memcpy(arr, &data, *size);
    return arr;
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
