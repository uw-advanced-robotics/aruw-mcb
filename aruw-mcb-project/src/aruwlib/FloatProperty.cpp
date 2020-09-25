#include "FloatProperty.hpp"

namespace aruwlib
{
void FloatProperty::serializeData(uint8_t* arr) const
{
    if (arr == nullptr)
    {
        return;
    }
    memcpy(arr, &data, sizeof(float));
}

void FloatProperty::setProperty(float data) { this->data = data; }

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

}  // namespace aruwlib
