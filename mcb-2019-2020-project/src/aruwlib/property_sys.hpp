#ifndef __PROPERTY_HPP__
#define __PROPERTY_HPP__
#include <memory>
#include <utility>
#include <rm-dev-board-a/board.hpp>
#include "src/aruwlib/communication/serial/dji_serial.hpp"
#include <modm/container.hpp>
#include <modm/architecture/driver/atomic/queue.hpp>

/**
 * Property Serial Protocol
 * Long Package
 * {
 * [PackageType(1 Byte)]
 * [Initial Sequence Number(1 Byte)]
 * [Expected Package Count(1 Byte)]
 * [Data]
 * 
 * }
 * 
 * Short Package
 * {
 * [Command(1 Byte)]
 * [Data]
 * }
 * 
 */
namespace aruwlib {

class PropertySystem : public aruwlib::serial::DJISerial
{
 public:
    typedef enum : uint8_t {
        UBYTE_PROPERTY = 1,
        USHORT_PROPERTY = 2,
        UINTEGER_PROPERTY = 3,
        BYTE_PROPERTY = 4,
        SHORT_PROPERTY = 5,
        INTEGER_PROPERTY = 6,
        FLOAT_PROPERTY = 7,
        BOOL_PROPERTY = 8
    } PropertyType;

    PropertySystem();

    void initializePropertySystem();

    /**
     * Process property query from serial and interactive with ROM.
     * Call periodically in order to make Property System be reactive.
     */
    void updatePropertySystem();

    /**
     * Add a data to PropertySystem
     * @param data pointer to data to be managed
     * @param property_name name of property
     * @param name_length length of property name
     * @return alias id of the property corresponding to given data
     */
    template<class Type>
    uint16_t addProperty(Type* data, uint8_t* property_name, uint8_t name_length);

    /**
     * Add a array to PropertySystem
     * @param array pointer to array to be managed
     * @param length length of array
     * @param property_name name of property
     * @param name_length length of property name
     * @return alias id of the property corresponding to given data
     */
    template<class Type>
    uint16_t addProperty(Type* array, uint16_t length, uint8_t* property_name, uint8_t name_length);
    /**
     * Send a property through serial
     * @param property_id
     * @return if the operation succeed
     */
    bool sendProperty(uint16_t property_id);
    /**
     * Send property table through serial
     * @return if the operation succeed
     */
    bool sendPropertyTable();

    /**
     * Send all properties through serial
     * @return if the operation succeed
     */
    bool sendAllProperty();

    /**
     * Check if number of properties in PropertySystem reaches maximum
     * @return if PropertySystem is full
     */
    bool isFull();

    template<class T>
    PropertyType typeToEnum(T* type);

    /**
     * Handle serial message received
     * @param completeMessage serial message received
     */
    void messageReceiveCallback(SerialMessage_t completeMessage) override;

 private:
    static const uint16_t PROPERTY_MESSAGE_TYPE_LONG_PACKAGE = 0x1001;
    static const uint16_t PROPERTY_MESSAGE_TYPE_SHORT_PACKAGE = 0x1002;

    static const uint8_t LONG_PACKAGE_TYPE_DATA = 0x01;
    static const uint8_t LONG_PACKAGE_TYPE_TABLE_DATA = 0x02;
    static const uint8_t LONG_PACKAGE_TYPE_QUERY = 0x03;
    static const uint8_t LONG_PACKAGE_TYPE_TABLE_QUERY = 0x04;
    // static const uint8_t LONG_PACKAGE_TYPE_SAVE_PROPERTY = 0x05;
    // static const uint8_t LONG_PACKAGE_TYPE_ADD_PROPERTY = 0x06;

    static const uint8_t SHORT_PACKAGE_TYPE_SEND_ALL_PROPERTY = 0x11;
    // static const uint8_t SHORT_PACKAGE_TYPE_SAVE_ALL_PROPERTY = 0x12;
    static const uint8_t SHORT_PACKAGE_TYPE_SEND_TABLE = 0x13;
    // static const uint8_t SHORT_PACKAGE_TYPE_SAVE_TABLE = 0x14;
    // static const uint8_t SHORT_PACKAGE_TYPE_DELETE_PROPERTY = 0x15;
    static const uint8_t SHORT_PACKAGE_TYPE_ADJUST_DEQUEUE_RATE = 0x16;

    static const uint8_t DEFAULT_DEQUEUE_RATE_TX_DATA = 200;
    static const uint8_t DEFAULT_DEQUEUE_RATE_TX_TABLE_DATA = 200;
    static const uint8_t DEFAULT_DEQUEUE_RATE_TX_LONG_PACKAGE = 2;
    static const uint8_t DEFAULT_DEQUEUE_RATE_TX_SHORT_PACKAGE = 2;

    static const uint16_t PROPERTY_TABLE_MAX_SIZE = 1024;

    static const uint16_t MAX_PACKAGE_DATA_LENGTH = 200;

    typedef struct {
        uint16_t id;
        PropertyType type;
        uint8_t* name;
        uint8_t nameLength;
        void* dataPointer;
        uint8_t typeSize;
        uint8_t byteCount;
    } Property_t;

    typedef struct {
        uint8_t packageType;
        uint8_t initialSequenceNumber;
        uint8_t expectedMessageCount;
        uint8_t data[MAX_PACKAGE_DATA_LENGTH];
        uint8_t dataLength;
    } LongPackage_t;

    typedef struct {
        uint8_t packageType;
        uint8_t* data;
        uint8_t dataLength;
    } ShortPackage_t;

    Property_t propertyTable[PROPERTY_TABLE_MAX_SIZE];
    uint16_t propertyTableSize;
    // Store Properties Waiting to be Sent
    modm::Queue<uint16_t, modm::LinkedList<uint16_t>> txDataQueue;

    // Maximum Number of property data to send in each call to updatePropertySystem()
    uint16_t txDataDequeueRate;

    // Store Property Table Entries Waiting to be Sent
    modm::Queue<uint16_t, modm::LinkedList<uint16_t>> txTableDataQueue;

    // Maximum Number of table entry to send in each call to updatePropertySystem()
    uint16_t txTableDataDequeueRate;

    modm::Queue<LongPackage_t*, modm::LinkedList<LongPackage_t*>> txLongPackageQueue;

    uint16_t txLongPackageDequeueRate;

    modm::Queue<ShortPackage_t*, modm::LinkedList<ShortPackage_t*>> txShortPackageQueue;

    uint16_t txShortPackageDequeueRate;

    /**
     * Send a property data through serial
     * @param property
     * @return if the operation succeed
     */
    bool sendPropertyData(Property_t* property);

    /**
     * Send property table through serial
     * @param property
     * @return if the operation succeed
     */
    bool sendPropertyTableEntry(Property_t* property);

    void dequeueTxDataQueue();
    void dequeueTxTableDataQueue();
    void dequeueTxLongPackageQueue();

    /**
     * Add data of given property to given package
     * @param property
     * @param package
     * @return if the operation succeed
     */
    bool packPropertyData(Property_t* property, LongPackage_t* package);

    /**
     * Send property table through serial
     * @param property
     * @return if the operation succeed
     */
    bool packPropertyTableEntry(Property_t* property, LongPackage_t* package);
};
}  // namespace aruwlib

#endif
