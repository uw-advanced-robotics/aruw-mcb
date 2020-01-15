#ifndef __PROPERTY_HPP__
#define __PROPERTY_HPP__
#include <rm-dev-board-a/board.hpp>
#include "src/aruwlib/communication/serial/dji_serial.hpp"
#include <modm/container.hpp>
#include <unordered_map>
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

class PropertySystem : aruwlib::serial::DJISerial
{

 public:

   typedef enum {
        BYTE_PROPERTY = 1,
        SHORT_PROPERTY = 2,
        INTEGER_PROPERTY = 3,
        FLOAT_PROPERTY = 4,
        BYTE_ARRAY_PROPERTY = 5
    } PropertyType;
    
    PropertySystem();
    ~PropertySystem();

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
    template <class Type>
    uint16_t addProperty(Type* data, uint8_t* property_name, uint8_t name_length);
    
    /**
     * Add a array to PropertySystem
     * @param array pointer to array to be managed
     * @param length length of array
     * @param property_name name of property
     * @param name_length length of property name
     * @return alias id of the property corresponding to given data
     */
    template <class Type>
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
    void sendAllProperty();

    /**
     * Check if number of properties in PropertySystem reaches maximum
     * @return if PropertySystem is full
     */
    bool isFull();

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
    //static const uint8_t LONG_PACKAGE_TYPE_SAVE_PROPERTY = 0x05;
    //static const uint8_t LONG_PACKAGE_TYPE_ADD_PROPERTY = 0x06;
    
    static const uint8_t SHORT_PACKAGE_TYPE_SEND_ALL_PROPERTY = 0x11;
    //static const uint8_t SHORT_PACKAGE_TYPE_SAVE_ALL_PROPERTY = 0x12;
    static const uint8_t SHORT_PACKAGE_TYPE_SEND_TABLE = 0x13;
    //static const uint8_t SHORT_PACKAGE_TYPE_SAVE_TABLE = 0x14;
    //static const uint8_t SHORT_PACKAGE_TYPE_DELETE_PROPERTY = 0x15;
    static const uint8_t SHORT_PACKAGE_TYPE_ADJUST_DEQUEUE_RATE = 0x16;

    static const uint8_t DEFAULT_DEQUEUE_RATE_TX_DATA = 20;
    static const uint8_t DEFAULT_DEQUEUE_RATE_TX_TABLE_DATA = 20;

    static const uint16_t PROPERTY_TABLE_MAX_SIZE = 1024;

    static const uint16_t MAX_PACKAGE_DATA_LENGTH = 200;

    struct Property_t {
        uint16_t id;
        uint8_t* name;
        uint8_t nameLength;
        modm::SmartPointer data;
        uint8_t length;
        
    };

    typedef struct {
        uint8_t packageType;
        uint8_t initialSequenceNumber;
        uint8_t expectedPackageCount;
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
    //Store Properties Waiting to be Sent
    modm::Queue<Property_t*, modm::LinkedList<Property_t*>> txDataQueue;
    
    //Maximum Number of property data to send in each call to updatePropertySystem()
    uint16_t txDataDequeueRate;

    //Store Property Table Entries Waiting to be Sent
    modm::Queue<Property_t*, modm::LinkedList<Property_t*>> txTableDataQueue;

    //Maximum Number of table entry to send in each call to updatePropertySystem()
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

}

#endif