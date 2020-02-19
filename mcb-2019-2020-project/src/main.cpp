#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>

#include "src/aruwlib/control/controller_mapper.hpp"
#include "src/aruwsrc/control/blink_led_command.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwsrc/control/example_command.hpp"
#include "src/aruwsrc/control/example_subsystem.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"
#include "src/aruwlib/algorithms/contiguous_float_test.hpp"
#include "src/aruwlib/communication/serial/ref_serial.hpp"
#include "src/osdklib/dji_vehicle.hpp"
#include "src/osdklib/dji_type.hpp"
#include "src/osdklib/dji_telemetry.hpp"
aruwsrc::control::ExampleSubsystem testSubsystem;

aruwlib::serial::RefSerial refereeSerial;

using namespace aruwlib::sensors;
RecvContainer* receivedFrame;
Vehicle vehicle = Vehicle(false);

void
poll_Drone_Data(void)
{
  receivedFrame = vehicle.protocolLayer->receive();
  if (receivedFrame->recvInfo.cmd_id != 0xFF)
  {
    vehicle.processReceivedData(receivedFrame);
  }
}

void
poll_Drone_Data(uint16_t times)
{
  for (size_t i = 0; i < times; i++)
  {
      receivedFrame = vehicle.protocolLayer->receive();
    if (receivedFrame->recvInfo.cmd_id != 0xFF)
    {
        vehicle.processReceivedData(receivedFrame);
    }
  }
}

using namespace DJI::OSDK::Telemetry;
void
userActivate()
{
  //! At your DJI developer account look for: app_key and app ID

  static char key_buf[65] = "48c9dc05511e783d6447b0dc45372f7cc20cf43bfeced5635702934594267f02";

  DJI::OSDK::Vehicle::ActivateData user_act_data;
  user_act_data.ID = 1084430; /*your app ID here*/

  user_act_data.encKey = key_buf;

  vehicle.activate(&user_act_data);
}

TypeMap<TOPIC_STATUS_FLIGHT>::type flightStatus;
TypeMap<TOPIC_GPS_FUSED>::type     latLon;
TypeMap<TOPIC_RC>::type            rc;
TypeMap<TOPIC_VELOCITY>::type      velocity;
TypeMap<TOPIC_QUATERNION>::type    quaternion;

using namespace DJI::OSDK::Telemetry;
bool
subscribeToData()
{

  // Counters
  int elapsedTimeInMs = 0;
  int timeToPrintInMs = 1;

  // We will subscribe to six kinds of data:
  // 1. Flight Status at 1 Hz
  // 2. Fused Lat/Lon at 10Hz
  // 3. Fused Altitude at 10Hz
  // 4. RC Channels at 50 Hz
  // 5. Velocity at 50 Hz
  // 6. Quaternion at 200 Hz

  // Package 0: Subscribe to flight status at freq 1 Hz
  int       pkgIndex        = 0;
  int       freq            = 1;
  TopicName topicList1Hz[]  = { TOPIC_STATUS_FLIGHT };
  int       numTopic        = sizeof(topicList1Hz) / sizeof(topicList1Hz[0]);
  bool      enableTimestamp = false;

  bool pkgStatus = vehicle.subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList1Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  vehicle.subscribe->startPackage(pkgIndex);
  modm::delayMilliseconds(10);
  /*ack = waitForACK();
  if(ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, func);

    // Cleanup
    vehicle.subscribe->removePackage(pkgIndex);
    ack = waitForACK();
    if(ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }

    return false;
  }*/

  // Package 1: Subscribe to Lat/Lon, and Alt at freq 10 Hz
  pkgIndex                  = 1;
  freq                      = 10;
  TopicName topicList10Hz[] = { TOPIC_GPS_FUSED };
  numTopic                  = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  enableTimestamp           = false;

  pkgStatus = vehicle.subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  vehicle.subscribe->startPackage(pkgIndex);
  modm::delayMilliseconds(10);
  /*ack = waitForACK();
  if(ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, func);

    // Cleanup
    vehicle.subscribe->removePackage(pkgIndex);
    ack = waitForACK();
    if(ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }

    return false;
  }*/

  // Package 2: Subscribe to RC Channel and Velocity at freq 50 Hz
  pkgIndex                  = 2;
  freq                      = 50;
  TopicName topicList50Hz[] = { TOPIC_RC, TOPIC_VELOCITY };
  numTopic                  = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
  enableTimestamp           = false;

  pkgStatus = vehicle.subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  vehicle.subscribe->startPackage(pkgIndex);
  modm::delayMilliseconds(10);
  /*ack = waitForACK();
  if(ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, func);

    // Cleanup
    vehicle.subscribe->removePackage(pkgIndex);
    ack = waitForACK();
    if(ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }

    return false;
  }*/

  // Package 3: Subscribe to Quaternion at freq 200 Hz.
  pkgIndex                   = 3;
  freq                       = 200;
  TopicName topicList200Hz[] = { TOPIC_QUATERNION };
  numTopic        = sizeof(topicList200Hz) / sizeof(topicList200Hz[0]);
  enableTimestamp = false;

  pkgStatus = vehicle.subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList200Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  vehicle.subscribe->startPackage(pkgIndex);
  modm::delayMilliseconds(10);
  /*ack = waitForACK();
  if(ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, func);

    // Cleanup
    vehicle.subscribe->removePackage(pkgIndex);
    ack = waitForACK();
    if(ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }

    return false;
  }*/

  // Wait for the data to start coming in.
  modm::delayMilliseconds(20);
  return true;
}

void updateValue() {
    flightStatus = vehicle.subscribe->getValue<TOPIC_STATUS_FLIGHT>();
    latLon       = vehicle.subscribe->getValue<TOPIC_GPS_FUSED>();
    rc           = vehicle.subscribe->getValue<TOPIC_RC>();
    velocity     = vehicle.subscribe->getValue<TOPIC_VELOCITY>();
    quaternion   = vehicle.subscribe->getValue<TOPIC_QUATERNION>();
}
int main()
{
    aruwlib::algorithms::ContiguousFloatTest contiguousFloatTest;
    contiguousFloatTest.testCore();
    contiguousFloatTest.testBadBounds();
    contiguousFloatTest.testDifference();
    contiguousFloatTest.testRotationBounds();
    contiguousFloatTest.testShiftingValue();
    contiguousFloatTest.testWrapping();

    Board::initialize();
    aruwlib::Remote::initialize();

    refereeSerial.initialize();

    Mpu6500::init();

    modm::SmartPointer testDefaultCommand(
        new aruwsrc::control::ExampleCommand(&testSubsystem));

    CommandScheduler::registerSubsystem(&testSubsystem);

    modm::SmartPointer blinkCommand(
        new aruwsrc::control::BlinkLEDCommand(&testSubsystem));

    // timers
    // arbitrary, taken from last year since this send time doesn't overfill
    // can bus
    modm::ShortPeriodicTimer motorSendPeriod(3);
    // update imu
    modm::ShortPeriodicTimer updateImuPeriod(2);

    IoMapper::addToggleMapping(
        IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP, {}),
        blinkCommand
    );

    vehicle.protocolLayer->getDriver()->init();
    vehicle.protocolLayer->getDriver()->getDeviceStatus();
    modm::delayMilliseconds(10);
    vehicle.functionalSetUp();
    poll_Drone_Data(100);
    vehicle.subscribe->verify();
    poll_Drone_Data(100);
    vehicle.obtainCtrlAuthority();
    poll_Drone_Data(100);
    userActivate();
    poll_Drone_Data(100);
    subscribeToData();
    poll_Drone_Data(100);


    while (1)
    {
        /*
        // do this as fast as you can
        aruwlib::can::CanRxHandler::pollCanData();
        refereeSerial.updateSerial();

        aruwlib::Remote::read();

        if (updateImuPeriod.execute())
        {
            Mpu6500::read();
        }

        if (motorSendPeriod.execute())
        {
            aruwlib::control::CommandScheduler::run();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }
        */
        poll_Drone_Data();
        updateValue();
        modm::delayMicroseconds(10);
    }
    return 0;
}
