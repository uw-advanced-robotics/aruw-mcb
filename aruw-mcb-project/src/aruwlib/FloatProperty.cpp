#include "FloatProperty.hpp"

#include <iomanip>
#include <sstream>

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

std::string FloatProperty::toString() const
{
    std::stringstream ss;
    ss << data << std::setprecision(4);
    return ss.str();
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
