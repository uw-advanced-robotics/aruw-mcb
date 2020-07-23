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
    arr[0] = static_cast<uint8_t>(getPropertyType());
    arr[1] = (getSerializationArrSize() >> 8) & 0xff;
    arr[2] = getSerializationArrSize() & 0xff;
    uint16_t propertyNameStrLen = strlen(propertyName);
    memcpy(arr + BASE_PROPERTY_HEADER_LENGTH, propertyName, propertyNameStrLen);
    serializeData(arr + BASE_PROPERTY_HEADER_LENGTH + propertyNameStrLen);
}
}  // namespace aruwlib