#include "BaseProperty.hpp"

namespace aruwlib
{
uint16_t BaseProperty::getFullSerializationSize() const
{
    return BASE_PROPERTY_HEADER_LENGTH + getSerializationArrSize();
}

void BaseProperty::fullSerialization(uint8_t *arr) const
{
    if (arr == nullptr)
    {
        return;
    }
    arr[0] = getPropertyType();
    arr[1] = (getSerializationArrSize() >> 8) & 0xff;
    arr[2] = getSerializationArrSize() & 0xff;
    serializeData(arr + BASE_PROPERTY_HEADER_LENGTH);
}
}  // namespace aruwlib