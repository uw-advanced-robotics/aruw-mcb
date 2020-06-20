#include "BoolProperty.hpp"

namespace aruwlib
{
uint8_t* BoolProperty::serializeData(uint16_t* size) const
{
    *size = sizeof(bool);
    uint8_t* arr = new uint8_t[*size];
    arr[0] = data;
    return arr;
}

bool BoolProperty::setProperty(void* data)
{
    if (data == nullptr)
    {
        return false;
    }
    this->data = (bool*)(&data);
    return true;
}

BoolProperty& BoolProperty::operator=(bool p2)
{
    data = p2;
    return *this;
}

BoolProperty& BoolProperty::operator&=(BoolProperty& p1)
{
    data &= p1.data;
    return *this;
}

BoolProperty& BoolProperty::operator&=(bool p1)
{
    data &= p1;
    return *this;
}

BoolProperty& BoolProperty::operator|=(BoolProperty& p1)
{
    data |= p1.data;
    return *this;
}

BoolProperty& BoolProperty::operator|=(bool p1)
{
    data |= p1;
    return *this;
}

};  // namespace aruwlib
