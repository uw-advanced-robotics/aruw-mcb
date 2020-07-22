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

bool FloatProperty::setProperty(void* data)
{
    if (data == nullptr)
    {
        return false;
    }
    this->data = *reinterpret_cast<float*>(data);
    return true;
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

}  // namespace aruwlib
