// #include "property_sys_serial.hpp"


// namespace aruwlib
// {

// PropertySystem::PropertySystem() :
// DJISerial(SerialPort::PORT_UART2, false),
// propertyTableSize(0),
// txDataQueue(),
// txDataDequeueRate(DEFAULT_DEQUEUE_RATE_TX_DATA),
// txTableDataQueue(),
// txTableDataDequeueRate(DEFAULT_DEQUEUE_RATE_TX_TABLE_DATA),
// txLongPackageDequeueRate(DEFAULT_DEQUEUE_RATE_TX_LONG_PACKAGE),
// txShortPackageDequeueRate(DEFAULT_DEQUEUE_RATE_TX_SHORT_PACKAGE)
// {
// }

// void PropertySystem::initializePropertySystem()
// {
//     this->initialize();
// }

// // cppcheck-suppress unusedParameters //
// template<class T>
// PropertySystem::PropertyType PropertySystem::typeToEnum(T* type __attribute__((unused))) {
//     if (std::is_same<T, uint8_t>::value)
//     {
//         return UBYTE_PROPERTY;
//     }
//     if (std::is_same<T, uint16_t>::value)
//     {
//         return USHORT_PROPERTY;
//     }
//     if (std::is_same<T, uint32_t>::value)
//     {
//         return UINTEGER_PROPERTY;
//     }
//     if (std::is_same<T, int8_t>::value)
//     {
//         return BYTE_PROPERTY;
//     }
//     if (std::is_same<T, int16_t>::value)
//     {
//         return SHORT_PROPERTY;
//     }
//     if (std::is_same<T, int32_t>::value)
//     {
//         return INTEGER_PROPERTY;
//     }
//     if (std::is_same<T, float>::value)
//     {
//         return FLOAT_PROPERTY;
//     }
//     if (std::is_same<T, bool>::value)
//     {
//         return BOOL_PROPERTY;
//     }
// }

// template <class Type>
// PropertySystem::property_id_t PropertySystem::addProperty(Type *data, std::string propertyName)
// {
//     Property_t property;
//     property.name = propertyName;
//     property.id = propertyTableSize;
//     property.dataPointer = data;
//     property.type = typeToEnum(data);
//     property.byteCount = sizeof(Type);
//     property.typeSize = sizeof(Type);
//     propertyTable[property.id] = property;
//     propertyTableSize += 1;
//     return property.id;
// }


// template <class Type>
// PropertySystem::property_id_t PropertySystem::addArrayProperty(Type *data,
//                                     uint16_t length,
//                                     std::string propertyName)
// {
//     Property_t property;
//     property.name = propertyName;
//     property.id = propertyTableSize;
//     property.dataPointer = data;
//     property.type = typeToEnum(data);
//     property.byteCount = length * sizeof(Type);
//     property.typeSize = sizeof(Type);
//     propertyTable[property.id] = property;
//     propertyTableSize += 1;
//     return property.id;
// }

// bool PropertySystem::sendProperty(uint16_t propertyID)
// {
//     return propertyID < propertyTableSize ? txDataQueue.push(propertyID) : false;
// }

// bool PropertySystem::sendAllPropertyData()
// {
//     for (uint16_t i = 0; i < PROPERTY_TABLE_MAX_SIZE; i++)
//     {
//         if(!txDataQueue.push(i)) {
//             return false;
//         }
//     }
//     return true;
// }

// bool PropertySystem::sendAllPropertyTableEntry()
// {
//     for (uint16_t i = 0; i < PROPERTY_TABLE_MAX_SIZE; i++)
//     {
//         if(!txTableDataQueue.push(i)) {
//             return false;
//         }
//     }
//     return true;
// }

// void PropertySystem::updatePropertySystem()
// {
//     dequeueTxDataQueue();
//     dequeueTxTableDataQueue();
//     dequeueTxLongPackageQueue();
// }

// void PropertySystem::dequeueTxDataQueue() {
//     if (txDataQueue.isNotEmpty()){
//         LongPackage_t *txLongPackage = new LongPackage_t();
//         txLongPackage->packageType = LONG_PACKAGE_TYPE_DATA;
//         uint16_t i = 0;
//         while (txDataQueue.isNotEmpty() && i < txDataDequeueRate)
//         {
//             if (!packPropertyData(&propertyTable[txDataQueue.get()], txLongPackage))
//             {
//                 txLongPackageQueue.push(std::as_const(txLongPackage));
//                 txLongPackage = new LongPackage_t();
//                 txLongPackage->packageType = LONG_PACKAGE_TYPE_TABLE_DATA;
//                 packPropertyData(&propertyTable[txDataQueue.get()], txLongPackage);
//             }
//             txDataQueue.pop();
//             i += 1;
//         }
//         txLongPackageQueue.push(std::as_const(txLongPackage));
//     }
// }

// void PropertySystem::dequeueTxTableDataQueue() {
//     if (txTableDataQueue.isNotEmpty()){
//         LongPackage_t *txLongPackage = new LongPackage_t();
//         txLongPackage->packageType = LONG_PACKAGE_TYPE_TABLE_DATA;
//         uint16_t i = 0;
//         while (txTableDataQueue.isNotEmpty() && i < txTableDataDequeueRate)
//         {
//             if (!packPropertyTableEntry(&propertyTable[txTableDataQueue.get()], txLongPackage))
//             {
//                 txLongPackageQueue.push(std::as_const(txLongPackage));
//                 txLongPackage = new LongPackage_t();
//                 txLongPackage->packageType = LONG_PACKAGE_TYPE_TABLE_DATA;
//                 packPropertyTableEntry(&propertyTable[txTableDataQueue.get()], txLongPackage);
//             }
//             txTableDataQueue.pop();
//             i += 1;
//         }
//         txLongPackageQueue.push(std::as_const(txLongPackage));
//     }
// }


// bool PropertySystem::packPropertyData(Property_t *property, LongPackage_t *package)
// {
//     if (sizeof(property->id) +
//         sizeof(property->byteCount) +
//         property->byteCount >
//         (unsigned)(MAX_PACKAGE_DATA_LENGTH - package->dataLength))
//     {
//         return false;
//     }
//     uint8_t *currentPackData = package->data + package->dataLength;
//     *reinterpret_cast<int16_t *>(currentPackData + 1) = property->id;
//     currentPackData += sizeof(property->id);
//     *currentPackData = property->byteCount;
//     currentPackData += sizeof(property->byteCount);
//     memcpy(currentPackData,
//            property->dataPointer,
//            property->byteCount);
//     package->dataLength += sizeof(property->id) +
//                             sizeof(property->byteCount) +
//                             property->byteCount;
//     return true;
// }

// bool PropertySystem::packPropertyTableEntry(Property_t *property, LongPackage_t *package)
// {
//     if (sizeof(property->id) +
//         sizeof(property->byteCount) +
//         sizeof(property->type) +
//         property->name.length() >
//         (unsigned)(MAX_PACKAGE_DATA_LENGTH - package->dataLength))
//     {
//         return false;
//     }
//     uint8_t *currentPackData = package->data + package->dataLength;
//     *reinterpret_cast<int16_t *>(currentPackData + 1) = property->id;
//     currentPackData += sizeof(property->id);
//     currentPackData[0] = property->byteCount;
//     currentPackData[1] = property->type;
//     currentPackData[2] = property->name.length();
//     currentPackData += sizeof(property->byteCount) +
//                         sizeof(property->type) +
//                         sizeof(property->name.length());
//     memcpy(currentPackData,
//            property->name.c_str(),
//            property->name.length());
//     package->dataLength += sizeof(property->id) +
//                             sizeof(property->byteCount) +
//                             sizeof(property->type) +
//                             sizeof(property->name.length()) +
//                             property->name.length();
//     return true;
// }

// void PropertySystem::dequeueTxLongPackageQueue() {
//     if (txLongPackageQueue.isNotEmpty()){
//         LongPackage_t *txLongPackage;
//         uint16_t i = 0;
//         while (txLongPackageQueue.isNotEmpty() && i < txLongPackageDequeueRate)
//         {
//             txLongPackage = txLongPackageQueue.get();
//             this->txMessage.type = PROPERTY_MESSAGE_TYPE_LONG_PACKAGE;
//             this->txMessage.length = txLongPackage->dataLength +
//                                         sizeof(txLongPackage->packageType) +
//                                         sizeof(txLongPackage->initialSequenceNumber) +
//                                         sizeof(txLongPackage->expectedMessageCount);
//             this->txMessage.data[0] = txLongPackage->packageType;
//             this->txMessage.data[1] = txLongPackage->initialSequenceNumber;
//             this->txMessage.data[2] = txLongPackage->expectedMessageCount;

//             memcpy(this->txMessage.data + 3,
//                 txLongPackage->data,
//                 txLongPackage->dataLength);
//             this->send();
//             txLongPackageQueue.pop();
//             i += 1;
//         }
//     }
// }

// void PropertySystem::messageReceiveCallback(SerialMessage completeMessage)
// {
//     switch (completeMessage.type)
//     {
//         case PROPERTY_MESSAGE_TYPE_LONG_PACKAGE:
//         {
//             uint8_t packageType = completeMessage.data[0];
//             uint8_t initialSequenceNumber __attribute__((unused)) = completeMessage.data[1];
//             uint8_t expectedMessageCount __attribute__((unused)) = completeMessage.data[2];
//             uint8_t *currentAddress = completeMessage.data + 3;
//             switch (packageType)
//             {
//                 case LONG_PACKAGE_TYPE_DATA:
//                 {
//                     while (currentAddress - completeMessage.data < completeMessage.length)
//                     {
//                         PropertySystem::property_id_t id =
//                                 *reinterpret_cast<property_id_t*>(currentAddress);
//                         currentAddress += sizeof(id);
//                         uint8_t dataLength = currentAddress[0];
//                         currentAddress += sizeof(dataLength);
//                         if (propertyTable[id].byteCount == dataLength)
//                         {
//                             memcpy(propertyTable[id].dataPointer,
//                                 currentAddress,
//                                 dataLength);
//                         }
//                         currentAddress += dataLength;
//                     }
//                     break;
//                 }
//                 case LONG_PACKAGE_TYPE_QUERY:
//                 {
//                     while (currentAddress - completeMessage.data < completeMessage.length)
//                     {
//                         property_id_t id = *reinterpret_cast<property_id_t*>(currentAddress);
//                         currentAddress += sizeof(id);
//                         if (id < propertyTableSize)
//                         {
//                             txDataQueue.push(id);
//                         }
//                     }
//                     break;
//                 }
//                 case LONG_PACKAGE_TYPE_TABLE_QUERY:
//                 {
//                     while (currentAddress - completeMessage.data < completeMessage.length)
//                     {
//                         property_id_t id = *reinterpret_cast<property_id_t*>(currentAddress);
//                         currentAddress += sizeof(id);
//                         if (id < propertyTableSize)
//                         {
//                             txTableDataQueue.push(id);
//                         }
//                     }
//                     break;
//                 }
//                 default: {
//                     break;
//                 }
//             }
//             break;
//         }
//         case PROPERTY_MESSAGE_TYPE_SHORT_PACKAGE:
//         {
//             uint8_t packageType = completeMessage.data[0];
//             switch (packageType)
//             {
//                 case SHORT_PACKAGE_TYPE_SEND_TABLE:
//                 {
//                     sendAllPropertyTableEntry();
//                     break;
//                 }
//                 case SHORT_PACKAGE_TYPE_SEND_ALL_PROPERTY:
//                 {
//                     sendAllPropertyData();
//                     break;
//                 }
//                 default:
//                     break;
//             }
//             break;
//         }
//     default:
//         break;
//     }
// }

// template PropertySystem::property_id_t PropertySystem::addProperty<uint8_t>(uint8_t* data,
//                                                         std::string propertyName);
// template PropertySystem::property_id_t PropertySystem::addProperty<uint16_t>(uint16_t* data,
//                                                         std::string propertyName);
// template PropertySystem::property_id_t PropertySystem::addProperty<uint32_t>(uint32_t* data,
//                                                         std::string propertyName);
// template PropertySystem::property_id_t PropertySystem::addProperty<int8_t>(int8_t* data,
//                                                         std::string propertyName);
// template PropertySystem::property_id_t PropertySystem::addProperty<int16_t>(int16_t* data,
//                                                         std::string propertyName);
// template PropertySystem::property_id_t PropertySystem::addProperty<int32_t>(int32_t* data,
//                                                         std::string propertyName);
// template PropertySystem::property_id_t PropertySystem::addProperty<float>(float* data,
//                                                         std::string propertyName);
// template PropertySystem::property_id_t PropertySystem::addProperty<bool>(bool* data,
//                                                         std::string propertyName);

// template PropertySystem::property_id_t PropertySystem::addArrayProperty<uint8_t>(uint8_t* array,
//                                                         uint16_t length,
//                                                         std::string propertyName);
// template PropertySystem::property_id_t PropertySystem::addArrayProperty<uint16_t>(uint16_t* array,
//                                                         uint16_t length,
//                                                         std::string propertyName);
// template PropertySystem::property_id_t PropertySystem::addArrayProperty<uint32_t>(uint32_t* array,
//                                                         uint16_t length,
//                                                         std::string propertyName);
// template PropertySystem::property_id_t PropertySystem::addArrayProperty<int8_t>(int8_t* array,
//                                                         uint16_t length,
//                                                         std::string propertyName);
// template PropertySystem::property_id_t PropertySystem::addArrayProperty<int16_t>(int16_t* array,
//                                                         uint16_t length,
//                                                         std::string propertyName);
// template PropertySystem::property_id_t PropertySystem::addArrayProperty<int32_t>(int32_t* array,
//                                                         uint16_t length,
//                                                         std::string propertyName);
// template PropertySystem::property_id_t PropertySystem::addArrayProperty<float>(float* array,
//                                                         uint16_t length,
//                                                         std::string propertyName);
// template PropertySystem::property_id_t PropertySystem::addArrayProperty<bool>(bool* array,
//                                                         uint16_t length,
//                                                         std::string propertyName);

// }  // namespace aruwlib
