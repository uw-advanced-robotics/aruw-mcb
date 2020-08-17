#include "ContiguousFloatProperty.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>

namespace aruwlib
{
void ContiguousFloatProperty::serializeData(uint8_t* arr) const
{
    if (arr == nullptr)
    {
        return;
    }
    float val = data.getValue();
    float min = data.getLowerBound();
    float max = data.getUpperBound();
    memcpy(arr, &val, sizeof(float));
    memcpy(arr + sizeof(float), &min, sizeof(float));
    memcpy(arr + (2 * sizeof(float)), &max, sizeof(float));
}

std::string ContiguousFloatProperty::toString() const
{
    // value [minVal, maxVal]
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << data.getValue() << " [" << data.getLowerBound()
       << ", " << data.getUpperBound() << "]";
    return ss.str();
}

bool ContiguousFloatProperty::setProperty(void* data)
{
    if (data == nullptr)
    {
        return false;
    }
    this->data = *reinterpret_cast<aruwlib::algorithms::ContiguousFloat*>(data);
    return true;
}

ContiguousFloatProperty& ContiguousFloatProperty::operator=(
    aruwlib::algorithms::ContiguousFloat& other)
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

ContiguousFloatProperty& ContiguousFloatProperty::operator+=(
    aruwlib::algorithms::ContiguousFloat& other)
{
    float sum = this->data.getValue() + other.getValue();
    this->data.setValue(sum);
    return *this;
}

ContiguousFloatProperty& ContiguousFloatProperty::operator-=(
    aruwlib::algorithms::ContiguousFloat& other)
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

ContiguousFloatProperty& ContiguousFloatProperty::operator*=(
    aruwlib::algorithms::ContiguousFloat& other)
{
    float prod = this->data.getValue() * other.getValue();
    this->data.setValue(prod);
    return *this;
}

ContiguousFloatProperty& ContiguousFloatProperty::operator/=(
    aruwlib::algorithms::ContiguousFloat& other)
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
}  // namespace aruwlib