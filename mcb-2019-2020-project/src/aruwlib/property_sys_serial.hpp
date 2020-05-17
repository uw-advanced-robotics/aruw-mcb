// #ifndef PROPERTY_SYS_SERIAL_HPP_
// #define PROPERTY_SYS_SERIAL_HPP_

// #include <modm/container.hpp>

// #include "aruwlib/communication/serial/dji_serial.hpp"
// #include "property_sys.hpp"

// namespace aruwlib {
// class PropertySysSerial : public aruwlib::serial::DJISerial
// {
// public:
//     PropertySysSerial();

//     void messageReceiveCallback(const SerialMessage& completeMessage) override;

// private:
//     // Maximum number of properties in queues to process in each call to update
//     static const uint8_t DEFAULT_DEQUEUE_RATE_TX_DATA = 200;
//     static const uint8_t DEFAULT_DEQUEUE_RATE_TX_TABLE_DATA = 200;
//     static const uint8_t DEFAULT_DEQUEUE_RATE_TX_LONG_PACKAGE = 2;
//     static const uint8_t DEFAULT_DEQUEUE_RATE_TX_SHORT_PACKAGE = 2;

//     static const uint16_t MAX_PACKAGE_DATA_LENGTH = 200;

//     // serial header type
//     static const uint16_t PROPERTY_MESSAGE_TYPE_LONG_PACKAGE = 0x1001;
//     static const uint16_t PROPERTY_MESSAGE_TYPE_SHORT_PACKAGE = 0x1002;

//     // Behavior: MCB send data to PC or PC ask MCB to change data
//     static const uint8_t LONG_PACKAGE_TYPE_DATA = 0x01;

//     // Behavior: MCB send property table entries to PC
//     static const uint8_t LONG_PACKAGE_TYPE_TABLE_DATA = 0x02;

//     // Behavior: PC request MCB to send data with specific id
//     static const uint8_t LONG_PACKAGE_TYPE_QUERY = 0x03;

//     // Behavior: PC request MCB to send table entries with specific id
//     static const uint8_t LONG_PACKAGE_TYPE_TABLE_QUERY = 0x04;
//     // static const uint8_t LONG_PACKAGE_TYPE_SAVE_PROPERTY = 0x05;
//     // static const uint8_t LONG_PACKAGE_TYPE_ADD_PROPERTY = 0x06;

//     static const uint8_t SHORT_PACKAGE_TYPE_SEND_ALL_PROPERTY = 0x11;
//     // static const uint8_t SHORT_PACKAGE_TYPE_SAVE_ALL_PROPERTY = 0x12;
//     static const uint8_t SHORT_PACKAGE_TYPE_SEND_TABLE = 0x13;
//     // static const uint8_t SHORT_PACKAGE_TYPE_SAVE_TABLE = 0x14;
//     // static const uint8_t SHORT_PACKAGE_TYPE_DELETE_PROPERTY = 0x15;
//     static const uint8_t SHORT_PACKAGE_TYPE_ADJUST_DEQUEUE_RATE = 0x16;


//     /*
//      * Differ from short package, long package
//      * has initialSequenceNumber and expectedMessageCount
//      * and is usually used to carry larger data.
//      * 
//      * By checking sequence numbers of packages received
//      * with initialSequenceNumber and expectedMessageCount,
//      * the receiver could tell if package loss happen
//      * during some critical package sequence transmition
//      * like property table entries, and takes actions such as
//      * requesting sender to send property table again.
//      * 
//      * (Currently sequence number check is not implemented)
//      */
//     typedef struct {
//         uint8_t packageType;
//         uint8_t initialSequenceNumber;
//         uint8_t expectedMessageCount;
//         uint8_t data[MAX_PACKAGE_DATA_LENGTH];
//         uint8_t dataLength;
//     } LongPackage_t;

//     /*
//      * Short package don't have sequence number check mechanism,
//      * so it is usually used to hold short instructions.
//      */
//     typedef struct {
//         uint8_t packageType;
//         uint8_t* data;
//         uint8_t dataLength;
//     } ShortPackage_t;


//     // Store Properties Waiting to be Sent
//     modm::Queue<PropertySystem::property_id_t, modm::LinkedList<PropertySystem::property_id_t>> txDataQueue;

//     // Maximum Number of property data to send in each call to updatePropertySystem()
//     uint16_t txDataDequeueRate;

//     modm::Queue<LongPackage_t*, modm::LinkedList<LongPackage_t*>> txLongPackageQueue;

//     uint16_t txLongPackageDequeueRate;

//     modm::Queue<ShortPackage_t*, modm::LinkedList<ShortPackage_t*>> txShortPackageQueue;

//     uint16_t txShortPackageDequeueRate;

//     /**
//      * Send data of a property through serial
//      * @param property
//      * @return if the operation succeed
//      */
//     bool sendPropertyData(PropertySystem::Property* property);

//     /**
//      * Send a entry of property table through serial
//      * @param property
//      * @return if the operation succeed
//      */
//     bool sendPropertyTableEntry(PropertySystem::Property* property);


//     /**
//      * Add data of given property to given package
//      * @param property
//      * @param package
//      * @return if the operation succeed
//      */
//     bool packPropertyData(PropertySystem::Property* property, LongPackage_t* package);

//     /**
//      * Send property table through serial
//      * @param property
//      * @return if the operation succeed
//      */
//     bool packPropertyTableEntry(PropertySystem::Property* property, LongPackage_t* package);
// };

// }

// #endif  // PROPERTY_SYS_SERIAL_HPP_
