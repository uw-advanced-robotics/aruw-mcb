#include "BaseProperty.hpp"

namespace aruwlib
{
uint8_t *BaseProperty::fullSerialization() const
{
    uint16_t dataLength;
    uint8_t *data = serializeData(&dataLength);
    uint8_t *dataWithHeader = new uint8_t[BASE_PROPERTY_HEADER_LENGTH + dataLength];
    dataWithHeader[0] = getPropertyType();
    dataWithHeader[1] = (dataLength >> 8) & 0xff;
    dataWithHeader[2] = dataLength & 0xff;
    memcpy(dataWithHeader + BASE_PROPERTY_HEADER_LENGTH, data, dataLength);
    delete[] data;
    return dataWithHeader;
}

}  // namespace aruwlib