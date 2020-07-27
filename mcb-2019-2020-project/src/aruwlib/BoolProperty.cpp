#include "BoolProperty.hpp"

namespace aruwlib
{
void BoolProperty::serializeData(uint8_t* arr) const
{
    if (arr == nullptr)
    {
        return;
    }
    arr[0] = data;
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
