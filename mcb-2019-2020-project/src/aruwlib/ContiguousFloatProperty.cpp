#include "ContiguousFloatProperty.hpp"

namespace aruwlib
{
void ContiguousFloatProperty::serializeData(uint8_t* arr) const
{
    if (arr == nullptr)
    {
        return;
    }
    memcpy(arr, &data, sizeof(ContiguousFloat));
}

bool ContiguousFloatProperty::setProperty(void* data)
{
    if (data == nullptr)
    {
        return false;
    }
    this->data = *reinterpret_cast<ContiguousFloat*>(data);
    return true;
}

ContiguousFloatProperty& ContiguousFloatProperty::operator=(ContiguousFloat& other)
{
    this->data = other;
    return *this;
}

ContiguousFloatProperty& ContiguousFloatProperty::operator+=(ContiguousFloatProperty& other)
{
    float sum = this->data.getValue() + other.data.getValue();
    this->data.setValue(sum);
    return *this;
}

ContiguousFloatProperty& ContiguousFloatProperty::operator+=(ContiguousFloat& other)
{
    float sum = this->data.getValue() + other.getValue();
    this->data.setValue(sum);
    return *this;
}

ContiguousFloatProperty& ContiguousFloatProperty::operator-=(ContiguousFloat& other)
{
    float diff = this->data.getValue() - other.getValue();
    this->data.setValue(diff);
    return *this;
}

ContiguousFloatProperty& ContiguousFloatProperty::operator-=(ContiguousFloatProperty& other)
{
    float diff = this->data.getValue() - other.data.getValue();
    this->data.setValue(diff);
    return *this;
}

ContiguousFloatProperty& ContiguousFloatProperty::operator*=(ContiguousFloatProperty& other)
{
    float prod = this->data.getValue() * other.data.getValue();
    this->data.setValue(prod);
    return *this;
}

ContiguousFloatProperty& ContiguousFloatProperty::operator*=(ContiguousFloat& other)
{
    float prod = this->data.getValue() * other.getValue();
    this->data.setValue(prod);
    return *this;
}

ContiguousFloatProperty& ContiguousFloatProperty::operator/=(ContiguousFloat& other)
{
    float quot = this->data.getValue() / other.getValue();
    this->data.setValue(quot);
    return *this;
}

ContiguousFloatProperty& ContiguousFloatProperty::operator/=(ContiguousFloatProperty& other)
{
    float quot = this->data.getValue() / other.data.getValue();
    this->data.setValue(quot);
    return *this;
}
}  // namespace ARUW