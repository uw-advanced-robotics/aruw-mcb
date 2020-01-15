#include "src/aruwlib/property_sys.hpp"
#include <utility>

namespace aruwlib
{

PropertySystem::PropertySystem() : 
DJISerial(SerialPort::PORT_UART2, true),
propertyTableSize(0),
txDataQueue(),
txTableDataQueue(),
txDataDequeueRate(),
txTableDataDequeueRate(),
txLongPackageDequeueRate(),
txShortPackageDequeueRate()
{
}

void PropertySystem::initializePropertySystem()
{
    this->initialize()
}

template <class Type>
uint16_t PropertySystem::addProperty(Type *data, uint8_t *property_name, uint8_t name_length)
{
    Property_t property;
    property.name = property_name;
    property.nameLength = name_length;
    property.id = propertyTableSize;
    property.data = modm::SmartPointer(data);
    property.length = sizeof(Type);
    propertyTable[property.id] = property;
    propertyTableSize += 1;
    return property.id;
}

template <class Type>
uint16_t PropertySystem::addProperty(Type *data, uint16_t length, uint8_t *property_name, uint8_t name_length)
{
    Property_t property;
    property.name = property_name;
    property.nameLength = name_length;
    property.id = propertyTableSize;
    property.data = modm::SmartPointer(data);
    property.length = length * sizeof(Type);
    propertyTable[property.id] = property;
    propertyTableSize += 1;
    return property.id;
}

bool PropertySystem::sendProperty(uint16_t property_id)
{
    return txDataQueue.push(&propertyTable[property_id]);
}

bool PropertySystem::sendPropertyTable()
{
    bool flag = true;
    for (uint16_t i = 0; i < PROPERTY_TABLE_MAX_SIZE; i++)
    {
        flag = txTableDataQueue.push(&propertyTable[i]) ? flag : false;
    }
    return flag;
}

void PropertySystem::updatePropertySystem()
{
    LongPackage_t *txLongPackage;
    if (txDataQueue.isNotEmpty() ||
        txTableDataQueue.isNotEmpty())
    {
        txLongPackage = new LongPackage_t();
    }

    for (uint16_t i = 0; i < txDataDequeueRate && txDataQueue.isNotEmpty(); i++)
    {
        if (!packPropertyData(txDataQueue.get(), txLongPackage))
        {
            txLongPackageQueue.push(std::as_const(txLongPackage));
            txLongPackage = new LongPackage_t();
            packPropertyData(txDataQueue.get(), txLongPackage);
        }
        txTableDataQueue.pop();
    }
    txLongPackageQueue.push(std::as_const(txLongPackage));
    txLongPackage = new LongPackage_t();
    for (uint16_t i = 0; i < txTableDataDequeueRate && txTableDataQueue.isNotEmpty(); i++)
    {
        if (!packPropertyTableEntry(txTableDataQueue.get(), txLongPackage))
        {
            txLongPackageQueue.push(std::as_const(txLongPackage));
            txLongPackage = new LongPackage_t();
            packPropertyTableEntry(txTableDataQueue.get(), txLongPackage);
        }
        txTableDataQueue.pop();
    }

    for (uint16_t i = 0; i < txLongPackageDequeueRate && txLongPackageQueue.isNotEmpty(); i++)
    {
        txLongPackage = txLongPackageQueue.get();
        this->txMessage.length = txLongPackage->dataLength + 3;
        this->txMessage.data[0] = txLongPackage->packageType;
        this->txMessage.data[1] = txLongPackage->initialSequenceNumber;
        this->txMessage.data[2] = txLongPackage->expectedPackageCount;

        memcpy(this->txMessage.data + 3,
               txLongPackage->data,
               txLongPackage->dataLength);
        this->send();
        txTableDataQueue.pop();
    }
}

bool PropertySystem::packPropertyData(Property_t *property, LongPackage_t *package)
{
    if (property->length + 3 < MAX_PACKAGE_DATA_LENGTH - package->dataLength)
    {
        return false;
    }
    uint8_t *currentPackData = package->data + package->dataLength;
    *reinterpret_cast<int16_t *>(currentPackData) = property->id;
    currentPackData += 2;
    currentPackData[0] = property->length;
    currentPackData += 1;
    memcpy(currentPackData,
           property->data.getPointer(),
           property->length);
    return true;
}

bool PropertySystem::packPropertyTableEntry(Property_t *property, LongPackage_t *package)
{
    if (property->length + 4 + property->nameLength < MAX_PACKAGE_DATA_LENGTH - package->dataLength)
    {
        return false;
    }
    uint8_t *currentPackData = package->data + package->dataLength;
    *reinterpret_cast<int16_t *>(currentPackData) = property->id;
    currentPackData += 2;
    currentPackData[0] = property->length;
    currentPackData[1] = property->nameLength;
    currentPackData += 2;
    memcpy(currentPackData,
           property->name,
           property->nameLength);
    return true;
}

void PropertySystem::messageReceiveCallback(SerialMessage_t completeMessage)
{
    switch (completeMessage.type)
    {
        case PROPERTY_MESSAGE_TYPE_LONG_PACKAGE:
        {
            uint8_t packageType = completeMessage.data[0];
            uint8_t *currentAddress = completeMessage.data + 3;
            switch (packageType)
            {
                case LONG_PACKAGE_TYPE_DATA:
                {
                    while (currentAddress - completeMessage.data < completeMessage.length)
                    {
                        uint16_t id = *reinterpret_cast<uint16_t *>(currentAddress);
                        currentAddress += 2;
                        uint8_t dataLength = currentAddress[0];
                        currentAddress += 1;
                        if (propertyTable[id].length == dataLength)
                        {
                            memcpy(propertyTable[id].data.getPointer(),
                                currentAddress,
                                dataLength);
                        }
                        currentAddress += dataLength;
                    }
                    break;
                }
                case LONG_PACKAGE_TYPE_QUERY:
                {
                    while (currentAddress - completeMessage.data < completeMessage.length)
                    {
                        uint16_t id = *reinterpret_cast<uint16_t *>(currentAddress);
                        currentAddress += 2;
                        if (propertyTable[id].length != 0)
                        {
                            txDataQueue.push(&propertyTable[id]);
                        }
                    }
                    break;
                }
                case LONG_PACKAGE_TYPE_TABLE_QUERY:
                {
                    while (currentAddress - completeMessage.data < completeMessage.length)
                    {
                        uint16_t id = *reinterpret_cast<uint16_t *>(currentAddress);
                        currentAddress += 2;
                        if (propertyTable[id].length != 0)
                        {
                            txTableDataQueue.push(&propertyTable[id]);
                        }
                    }
                    break;
                }
                default: {
                    break;
                }
            }
            break;
        }
        case PROPERTY_MESSAGE_TYPE_SHORT_PACKAGE:
        {
            uint8_t packageType = completeMessage.data[0];
            switch (packageType)
            {
                case SHORT_PACKAGE_TYPE_SEND_TABLE:
                {
                    sendPropertyTable();
                    break;
                }
                case SHORT_PACKAGE_TYPE_SEND_ALL_PROPERTY:
                {
                    sendAllProperty();
                    break;
                }
                default:
                    break;
            }
            break;
        }
    default:
        break;
    }
}


} // namespace aruwlib