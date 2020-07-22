#include "Int32Property.hpp"

namespace aruwlib
{
void Int32Property::serializeData(uint8_t* arr) const
{
    if (arr == nullptr)
    {
        return;
    }
    memcpy(arr, &data, sizeof(int32_t));
}

bool Int32Property::setProperty(void* data)
{
    if (data == nullptr)
    {
        return false;
    }
    this->data = *reinterpret_cast<int32_t*>(data);
    return true;
}

Int32Property& Int32Property::operator=(int32_t other)
{
    data = other;
    return *this;
}

Int32Property& Int32Property::operator+=(Int32Property& other)
{
    data += other.data;
    return *this;
}

Int32Property& Int32Property::operator+=(int32_t other)
{
    data += other;
    return *this;
}

Int32Property& Int32Property::operator-=(Int32Property& other)
{
    data -= other.data;
    return *this;
}

Int32Property& Int32Property::operator-=(int32_t other)
{
    data -= other;
    return *this;
}

Int32Property& Int32Property::operator*=(Int32Property& other)
{
    data *= other.data;
    return *this;
}

Int32Property& Int32Property::operator*=(int32_t other)
{
    data *= other;
    return *this;
}

Int32Property& Int32Property::operator/=(Int32Property& other)
{
    data /= other.data;
    return *this;
}

Int32Property& Int32Property::operator/=(int32_t other)
{
    data /= other;
    return *this;
}

}  // namespace aruwlib
