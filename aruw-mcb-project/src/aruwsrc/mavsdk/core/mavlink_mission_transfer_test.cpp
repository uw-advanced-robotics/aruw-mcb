#include <algorithm>
#include <chrono>
#include <future>
#include <gtest/gtest.h>

#include "mavlink_mission_transfer.h"
#include "mocks/sender_mock.h"
#include "unused.h"

using namespace mavsdk;

using ::testing::_;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Truly;
using MockSender = NiceMock<mavsdk::testing::MockSender>;

using Result = MavlinkMissionTransfer::Result;
using ItemInt = MavlinkMissionTransfer::ItemInt;

static MAVLinkAddress own_address{42, 16};
static MAVLinkAddress target_address{99, MAV_COMP_ID_AUTOPILOT1};

static constexpr double timeout_s = 0.5;

#define ONCE_ONLY \
    static bool called = false; \
    EXPECT_FALSE(called); \
    called = true

class MavlinkMissionTransferTest : public ::testing::Test {
protected:
    MavlinkMissionTransferTest() :
        ::testing::Test(),
        timeout_handler(time),
        mmt(mock_sender, message_handler, timeout_handler, []() { return timeout_s; })
    {}

    void SetUp() override
    {
        ON_CALL(mock_sender, get_own_system_id()).WillByDefault(Return(own_address.system_id));
        ON_CALL(mock_sender, get_own_component_id())
            .WillByDefault(Return(own_address.component_id));
        ON_CALL(mock_sender, get_system_id()).WillByDefault(Return(target_address.system_id));
        ON_CALL(mock_sender, autopilot()).WillByDefault(Return(Sender::Autopilot::Px4));
    }

    MockSender mock_sender;
    MavlinkMessageHandler message_handler;
    FakeTime time;
    TimeoutHandler timeout_handler;
    MavlinkMissionTransfer mmt;
};

ItemInt make_item(uint8_t type, uint16_t sequence)
{
    ItemInt item;

    item.seq = sequence;
    item.frame = MAV_FRAME_MISSION;
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.current = uint8_t(sequence == 0 ? 1 : 0);
    item.autocontinue = 1;
    item.param1 = 1.0f;
    item.param2 = 2.0f;
    item.param3 = 3.0f;
    item.param4 = 4.0f;
    item.x = 5;
    item.y = 6;
    item.z = 7.0f;
    item.mission_type = type;

    return item;
}

TEST_F(MavlinkMissionTransferTest, UploadMissionDoesComplainAboutNoItems)
{
    std::vector<ItemInt> items;

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, [&prom](Result result) {
        EXPECT_EQ(result, Result::NoMissionAvailable);
        ONCE_ONLY;
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, UploadMissionDoesComplainAboutWrongSequence)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 2));

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, [&prom](Result result) {
        EXPECT_EQ(result, Result::InvalidSequence);
        ONCE_ONLY;
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, UploadMissionDoesComplainAboutInconsistentMissionTypesInAPI)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_FENCE, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_FENCE, 1));
    items.push_back(make_item(MAV_MISSION_TYPE_FENCE, 2));

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, [&prom](Result result) {
        EXPECT_EQ(result, Result::MissionTypeNotConsistent);
        ONCE_ONLY;
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, UploadMissionDoesComplainAboutInconsistentMissionTypesInItems)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_FENCE, 1));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 2));

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, [&prom](Result result) {
        EXPECT_EQ(result, Result::MissionTypeNotConsistent);
        ONCE_ONLY;
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, UploadMissionDoesComplainAboutNoCurrent)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 2));
    // Remove the current flag again.
    items[0].current = 0;

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, [&prom](Result result) {
        EXPECT_EQ(result, Result::CurrentInvalid);
        ONCE_ONLY;
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, UploadMissionDoesComplainAboutTwoCurrents)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 2));
    // Add another current.
    items[1].current = 1;

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, [&prom](Result result) {
        EXPECT_EQ(result, Result::CurrentInvalid);
        ONCE_ONLY;
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, UploadMissionDoesNotCrashIfCallbackIsNull)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(false));

    // Catch the empty case
    std::vector<ItemInt> items;
    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, nullptr);
    mmt.do_work();

    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));

    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, nullptr);
    mmt.do_work();

    // Catch the WrongSequence case as well.
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 3));
    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, nullptr);
    mmt.do_work();

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, UploadMissionReturnsConnectionErrorWhenSendMessageFails)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(false));

    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, [&prom](Result result) {
        EXPECT_EQ(result, Result::ConnectionError);
        ONCE_ONLY;
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    // We want to be sure a timeout is not still triggered later.
    time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
    timeout_handler.run_once();

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

bool is_correct_mission_send_count(uint8_t type, unsigned count, const mavlink_message_t& message)
{
    if (message.msgid != MAVLINK_MSG_ID_MISSION_COUNT) {
        return false;
    }

    mavlink_mission_count_t mission_count;
    mavlink_msg_mission_count_decode(&message, &mission_count);
    return (
        message.msgid == MAVLINK_MSG_ID_MISSION_COUNT && message.sysid == own_address.system_id &&
        message.compid == own_address.component_id &&
        mission_count.target_system == target_address.system_id &&
        mission_count.target_component == target_address.component_id &&
        mission_count.count == count && mission_count.mission_type == type);
}

TEST_F(MavlinkMissionTransferTest, UploadMissionSendsCount)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_FENCE, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_FENCE, 1));

    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    EXPECT_CALL(mock_sender, send_message(Truly([&items](const mavlink_message_t& message) {
                    return is_correct_mission_send_count(
                        MAV_MISSION_TYPE_FENCE, items.size(), message);
                })));

    mmt.upload_items_async(MAV_MISSION_TYPE_FENCE, items, [](Result result) {
        UNUSED(result);
        EXPECT_TRUE(false);
    });
    mmt.do_work();
}

TEST_F(MavlinkMissionTransferTest, UploadMissionResendsCount)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_FENCE, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_FENCE, 1));

    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    EXPECT_CALL(mock_sender, send_message(Truly([&items](const mavlink_message_t& message) {
                    return is_correct_mission_send_count(
                        MAV_MISSION_TYPE_FENCE, items.size(), message);
                })))
        .Times(2);

    mmt.upload_items_async(MAV_MISSION_TYPE_FENCE, items, [](Result result) {
        UNUSED(result);
        EXPECT_TRUE(false);
    });
    mmt.do_work();

    time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
    timeout_handler.run_once();
}

TEST_F(MavlinkMissionTransferTest, UploadMissionTimeoutAfterSendCount)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));

    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    EXPECT_CALL(mock_sender, send_message(Truly([&items](const mavlink_message_t& message) {
                    return is_correct_mission_send_count(
                        MAV_MISSION_TYPE_MISSION, items.size(), message);
                })))
        .Times(MavlinkMissionTransfer::retries);

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, [&prom](Result result) {
        EXPECT_EQ(result, Result::Timeout);
        ONCE_ONLY;
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(0)), std::future_status::timeout);

    // After the specified retries we should give up with a timeout.
    for (unsigned i = 0; i < MavlinkMissionTransfer::retries; ++i) {
        time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
        timeout_handler.run_once();
    }

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);
}

mavlink_message_t make_mission_request_int(uint8_t type, int sequence)
{
    mavlink_message_t message;
    mavlink_msg_mission_request_int_pack(
        own_address.system_id,
        own_address.component_id,
        &message,
        target_address.system_id,
        target_address.component_id,
        sequence,
        type);
    return message;
}

mavlink_message_t make_mission_ack(uint8_t type, uint8_t result)
{
    mavlink_message_t message;
    mavlink_msg_mission_ack_pack(
        own_address.system_id,
        own_address.component_id,
        &message,
        target_address.system_id,
        target_address.component_id,
        result,
        type);
    return message;
}

bool is_the_same_mission_item_int(const ItemInt& item, const mavlink_message_t& message)
{
    if (message.msgid != MAVLINK_MSG_ID_MISSION_ITEM_INT) {
        return false;
    }
    mavlink_mission_item_int_t mission_item_int;
    mavlink_msg_mission_item_int_decode(&message, &mission_item_int);

    return (
        message.sysid == own_address.system_id && //
        message.compid == own_address.component_id && //
        mission_item_int.target_system == target_address.system_id && //
        mission_item_int.target_component == target_address.component_id && //
        mission_item_int.seq == item.seq && //
        mission_item_int.frame == item.frame && //
        mission_item_int.command == item.command && //
        mission_item_int.current == item.current && //
        mission_item_int.autocontinue == item.autocontinue && //
        mission_item_int.param1 == item.param1 && //
        mission_item_int.param2 == item.param2 && //
        mission_item_int.param3 == item.param3 && //
        mission_item_int.param4 == item.param4 && //
        mission_item_int.x == item.x && //
        mission_item_int.y == item.y && //
        mission_item_int.z == item.z && //
        mission_item_int.mission_type == item.mission_type);
}

TEST_F(MavlinkMissionTransferTest, UploadMissionSendsMissionItems)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));

    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, [&prom](Result result) {
        EXPECT_EQ(result, Result::Success);
        ONCE_ONLY;
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_CALL(mock_sender, send_message(Truly([&items](const mavlink_message_t& message) {
                    return is_the_same_mission_item_int(items[0], message);
                })));

    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 0));

    EXPECT_CALL(mock_sender, send_message(Truly([&items](const mavlink_message_t& message) {
                    return is_the_same_mission_item_int(items[1], message);
                })));

    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 1));

    message_handler.process_message(
        make_mission_ack(MAV_MISSION_TYPE_MISSION, MAV_MISSION_ACCEPTED));

    // We are finished and should have received the successful result.
    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    // We do not expect a timeout later though.
    time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
    timeout_handler.run_once();

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, UploadMissionResendsMissionItems)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));

    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, [&prom](Result result) {
        EXPECT_EQ(result, Result::Success);
        ONCE_ONLY;
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_CALL(mock_sender, send_message(Truly([&items](const mavlink_message_t& message) {
                    return is_the_same_mission_item_int(items[0], message);
                })));

    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 0));

    EXPECT_CALL(mock_sender, send_message(Truly([&items](const mavlink_message_t& message) {
                    return is_the_same_mission_item_int(items[0], message);
                })));

    // Request 0 again in case it had not arrived.
    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 0));

    EXPECT_CALL(mock_sender, send_message(Truly([&items](const mavlink_message_t& message) {
                    return is_the_same_mission_item_int(items[1], message);
                })));

    // Request 1 finally.
    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 1));

    message_handler.process_message(
        make_mission_ack(MAV_MISSION_TYPE_MISSION, MAV_MISSION_ACCEPTED));

    // We are finished and should have received the successful result.
    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, UploadMissionResendsMissionItemsButGivesUpAfterSomeRetries)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_FENCE, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_FENCE, 1));

    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.upload_items_async(MAV_MISSION_TYPE_FENCE, items, [&prom](Result result) {
        EXPECT_EQ(result, Result::Timeout);
        ONCE_ONLY;
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_CALL(mock_sender, send_message(Truly([&items](const mavlink_message_t& message) {
                    return is_the_same_mission_item_int(items[0], message);
                })))
        .Times(MavlinkMissionTransfer::retries);

    for (unsigned i = 0; i < MavlinkMissionTransfer::retries; ++i) {
        message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 0));
    }

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(0)), std::future_status::timeout);

    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 0));

    // We are finished and should have received the successful result.
    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, UploadMissionAckArrivesTooEarly)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));

    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, [&prom](Result result) {
        EXPECT_EQ(result, Result::ProtocolError);
        ONCE_ONLY;
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_CALL(mock_sender, send_message(Truly([&items](const mavlink_message_t& message) {
                    return is_the_same_mission_item_int(items[0], message);
                })));

    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 0));

    // Don't request item 1 but already send ack.
    message_handler.process_message(
        make_mission_ack(MAV_MISSION_TYPE_MISSION, MAV_MISSION_ACCEPTED));

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

class MavlinkMissionTransferNackTests
    : public MavlinkMissionTransferTest,
      public ::testing::WithParamInterface<std::pair<uint8_t, Result>> {};

TEST_P(MavlinkMissionTransferNackTests, UploadMissionNackAreHandled)
{
    uint8_t mavlink_nack = std::get<0>(GetParam());
    Result mavsdk_nack = std::get<1>(GetParam());

    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));

    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, [&prom, &mavsdk_nack](Result result) {
        EXPECT_EQ(result, mavsdk_nack);
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_CALL(mock_sender, send_message(Truly([&items](const mavlink_message_t& message) {
                    return is_the_same_mission_item_int(items[0], message);
                })));

    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 0));

    // Send nack now.
    message_handler.process_message(make_mission_ack(MAV_MISSION_TYPE_MISSION, mavlink_nack));

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

INSTANTIATE_TEST_SUITE_P(
    MavlinkMissionTransferTests,
    MavlinkMissionTransferNackTests,
    ::testing::Values(
        std::make_pair(MAV_MISSION_ERROR, Result::ProtocolError),
        std::make_pair(MAV_MISSION_UNSUPPORTED_FRAME, Result::UnsupportedFrame),
        std::make_pair(MAV_MISSION_UNSUPPORTED, Result::Unsupported),
        std::make_pair(MAV_MISSION_NO_SPACE, Result::TooManyMissionItems),
        std::make_pair(MAV_MISSION_INVALID, Result::InvalidParam),
        std::make_pair(MAV_MISSION_INVALID_PARAM1, Result::InvalidParam),
        std::make_pair(MAV_MISSION_INVALID_PARAM2, Result::InvalidParam),
        std::make_pair(MAV_MISSION_INVALID_PARAM3, Result::InvalidParam),
        std::make_pair(MAV_MISSION_INVALID_PARAM4, Result::InvalidParam),
        std::make_pair(MAV_MISSION_INVALID_PARAM5_X, Result::InvalidParam),
        std::make_pair(MAV_MISSION_INVALID_PARAM6_Y, Result::InvalidParam),
        std::make_pair(MAV_MISSION_INVALID_PARAM7, Result::InvalidParam),
        std::make_pair(MAV_MISSION_INVALID_SEQUENCE, Result::InvalidSequence),
        std::make_pair(MAV_MISSION_DENIED, Result::Denied),
        std::make_pair(MAV_MISSION_OPERATION_CANCELLED, Result::Cancelled)));

TEST_F(MavlinkMissionTransferTest, UploadMissionTimeoutNotTriggeredDuringTransfer)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 2));

    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, [&prom](Result result) {
        EXPECT_EQ(result, Result::Success);
        ONCE_ONLY;
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_CALL(mock_sender, send_message(Truly([&items](const mavlink_message_t& message) {
                    return is_the_same_mission_item_int(items[0], message);
                })));

    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 0));

    // We almost use up the max timeout in each cycle.
    time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 0.8 * 1000.)));
    timeout_handler.run_once();

    EXPECT_CALL(mock_sender, send_message(Truly([&items](const mavlink_message_t& message) {
                    return is_the_same_mission_item_int(items[1], message);
                })));

    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 1));

    time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 0.8 * 1000.)));
    timeout_handler.run_once();

    EXPECT_CALL(mock_sender, send_message(Truly([&items](const mavlink_message_t& message) {
                    return is_the_same_mission_item_int(items[2], message);
                })));

    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 2));

    time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 0.8 * 1000.)));
    timeout_handler.run_once();

    message_handler.process_message(
        make_mission_ack(MAV_MISSION_TYPE_MISSION, MAV_MISSION_ACCEPTED));

    // We are finished and should have received the successful result.
    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, UploadMissionTimeoutAfterSendMissionItem)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));

    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, [&prom](Result result) {
        EXPECT_EQ(result, Result::Timeout);
        ONCE_ONLY;
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_CALL(mock_sender, send_message(Truly([&items](const mavlink_message_t& message) {
                    return is_the_same_mission_item_int(items[0], message);
                })));

    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 0));

    // Make sure single timeout does not trigger it yet.
    time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1000. + 250)));
    timeout_handler.run_once();

    EXPECT_EQ(fut.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);

    // But multiple do.
    for (unsigned i = 0; i < (MavlinkMissionTransfer::retries - 1); ++i) {
        time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1000.)));
        timeout_handler.run_once();
    }

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    // Ignore later (wrong) ack.
    message_handler.process_message(
        make_mission_ack(MAV_MISSION_TYPE_MISSION, MAV_MISSION_ACCEPTED));

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, UploadMissionDoesNotCrashOnRandomMessages)
{
    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 0));

    message_handler.process_message(
        make_mission_ack(MAV_MISSION_TYPE_MISSION, MAV_MISSION_ACCEPTED));

    time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
    timeout_handler.run_once();

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

bool is_correct_mission_ack(uint8_t type, uint8_t result, const mavlink_message_t& message)
{
    if (message.msgid != MAVLINK_MSG_ID_MISSION_ACK) {
        return false;
    }

    mavlink_mission_ack_t ack;
    mavlink_msg_mission_ack_decode(&message, &ack);
    return (
        message.sysid == own_address.system_id && message.compid == own_address.component_id &&
        ack.target_system == target_address.system_id &&
        ack.target_component == target_address.component_id && ack.type == result &&
        ack.mission_type == type);
}

bool is_correct_autopilot_mission_ack(
    uint8_t type, uint8_t result, uint8_t target_component, const mavlink_message_t& message)
{
    if (message.msgid != MAVLINK_MSG_ID_MISSION_ACK) {
        return false;
    }

    mavlink_mission_ack_t ack;
    mavlink_msg_mission_ack_decode(&message, &ack);
    return (
        message.sysid == own_address.system_id && message.compid == own_address.component_id &&
        ack.target_system == target_address.system_id && ack.target_component == target_component &&
        ack.type == result && ack.mission_type == type);
}

TEST_F(MavlinkMissionTransferTest, UploadMissionCanBeCancelled)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));

    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::promise<void> prom;
    auto fut = prom.get_future();

    auto transfer = mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, [&prom](Result result) {
        EXPECT_EQ(result, Result::Cancelled);
        ONCE_ONLY;
        prom.set_value();
    });
    mmt.do_work();

    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 0));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_ack(
                        MAV_MISSION_TYPE_MISSION, MAV_MISSION_OPERATION_CANCELLED, message);
                })));

    auto ptr = transfer.lock();
    EXPECT_TRUE(ptr);
    if (ptr) {
        ptr->cancel();
    }

    // We are finished and should have received the successful result.
    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    // We do not expect a timeout later though.
    time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
    timeout_handler.run_once();

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

mavlink_message_t make_mission_request(uint8_t type, int sequence)
{
    mavlink_message_t message;
    mavlink_msg_mission_request_pack(
        own_address.system_id,
        own_address.component_id,
        &message,
        target_address.system_id,
        target_address.component_id,
        sequence,
        type);
    return message;
}

TEST_F(MavlinkMissionTransferTest, UploadMissionNacksNonIntCase)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_FENCE, 0));

    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.upload_items_async(MAV_MISSION_TYPE_FENCE, items, [&prom](Result result) {
        EXPECT_EQ(result, Result::Success);
        ONCE_ONLY;
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_ack(
                        MAV_MISSION_TYPE_FENCE, MAV_MISSION_UNSUPPORTED, message);
                })))
        .Times(1);

    // First the non-int wrong case comes in.
    message_handler.process_message(make_mission_request(MAV_MISSION_TYPE_FENCE, 0));

    EXPECT_CALL(mock_sender, send_message(Truly([&items](const mavlink_message_t& message) {
                    return is_the_same_mission_item_int(items[0], message);
                })));

    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_FENCE, 0));
    message_handler.process_message(make_mission_ack(MAV_MISSION_TYPE_FENCE, MAV_MISSION_ACCEPTED));

    // We are finished and should have received the successful result.
    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    // We do not expect a timeout later though.
    time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
    timeout_handler.run_once();

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, UploadMissionWithProgress)
{
    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));

    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::promise<void> prom;
    auto fut = prom.get_future();

    size_t num_progress_received = 0;
    float last_progress_received = NAN;

    mmt.upload_items_async(
        MAV_MISSION_TYPE_MISSION,
        items,
        [&prom](Result result) {
            EXPECT_EQ(result, Result::Success);
            ONCE_ONLY;
            prom.set_value();
        },
        [&num_progress_received, &last_progress_received](float progress) {
            if (num_progress_received == 0) {
                EXPECT_EQ(progress, 0.0f);
            } else {
                EXPECT_GT(progress, last_progress_received);
            }
            ++num_progress_received;
            last_progress_received = progress;
        });
    mmt.do_work();

    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 0));
    message_handler.process_message(make_mission_request_int(MAV_MISSION_TYPE_MISSION, 1));
    message_handler.process_message(
        make_mission_ack(MAV_MISSION_TYPE_MISSION, MAV_MISSION_ACCEPTED));

    // We are finished and should have received the successful result.
    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    EXPECT_EQ(last_progress_received, 1.0f);
    EXPECT_GE(num_progress_received, 3);
    EXPECT_LE(num_progress_received, 10);

    // We do not expect a timeout later though.
    time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
    timeout_handler.run_once();

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, DownloadMissionSendsRequestList)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    mavlink_mission_request_list_t mission_request_list;
                    mavlink_msg_mission_request_list_decode(&message, &mission_request_list);

                    return (
                        message.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_LIST &&
                        message.sysid == own_address.system_id &&
                        message.compid == own_address.component_id &&
                        mission_request_list.target_system == target_address.system_id &&
                        mission_request_list.target_component == target_address.component_id &&
                        mission_request_list.mission_type == MAV_MISSION_TYPE_MISSION);
                })));

    mmt.download_items_async(
        MAV_MISSION_TYPE_MISSION, [](Result result, const std::vector<ItemInt>& items) {
            UNUSED(result);
            UNUSED(items);
            EXPECT_TRUE(false);
        });
    mmt.do_work();
}

bool is_correct_mission_request_list(uint8_t type, const mavlink_message_t& message)
{
    if (message.msgid != MAVLINK_MSG_ID_MISSION_REQUEST_LIST) {
        return false;
    }
    mavlink_mission_request_list_t mission_request_list;
    mavlink_msg_mission_request_list_decode(&message, &mission_request_list);

    return (
        message.sysid == own_address.system_id && //
        message.compid == own_address.component_id && //
        mission_request_list.target_system == target_address.system_id && //
        mission_request_list.target_component == target_address.component_id && //
        mission_request_list.mission_type == type);
}

TEST_F(MavlinkMissionTransferTest, DownloadMissionResendsRequestList)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_request_list(MAV_MISSION_TYPE_MISSION, message);
                })))
        .Times(2);

    mmt.download_items_async(
        MAV_MISSION_TYPE_MISSION, [](Result result, const std::vector<ItemInt>& items) {
            UNUSED(result);
            UNUSED(items);
            EXPECT_TRUE(false);
        });
    mmt.do_work();

    time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
    timeout_handler.run_once();
}

TEST_F(MavlinkMissionTransferTest, DownloadMissionResendsRequestListButGivesUpAfterSomeRetries)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_request_list(MAV_MISSION_TYPE_MISSION, message);
                })))
        .Times(MavlinkMissionTransfer::retries);

    std::promise<void> prom;
    auto fut = prom.get_future();
    mmt.download_items_async(
        MAV_MISSION_TYPE_MISSION, [&prom](Result result, const std::vector<ItemInt>& items) {
            UNUSED(items);
            EXPECT_EQ(result, Result::Timeout);
            ONCE_ONLY;
            prom.set_value();
        });
    mmt.do_work();

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(0)), std::future_status::timeout);

    for (unsigned i = 0; i < MavlinkMissionTransfer::retries; ++i) {
        time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
        timeout_handler.run_once();
    }

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

bool is_correct_mission_request_int(
    uint8_t type, unsigned sequence, const mavlink_message_t& message)
{
    if (message.msgid != MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
        return false;
    }

    mavlink_mission_request_int_t mission_request_int;
    mavlink_msg_mission_request_int_decode(&message, &mission_request_int);
    return (
        message.sysid == own_address.system_id && message.compid == own_address.component_id &&
        mission_request_int.target_system == target_address.system_id &&
        mission_request_int.target_component == target_address.component_id &&
        mission_request_int.seq == sequence && mission_request_int.mission_type == type);
}

bool is_correct_autopilot_mission_request_int(
    uint8_t type, unsigned sequence, uint8_t target_component, const mavlink_message_t& message)
{
    if (message.msgid != MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
        return false;
    }

    mavlink_mission_request_int_t mission_request_int;
    mavlink_msg_mission_request_int_decode(&message, &mission_request_int);
    return (
        message.sysid == own_address.system_id && message.compid == own_address.component_id &&
        mission_request_int.target_system == target_address.system_id &&
        mission_request_int.target_component == target_component &&
        mission_request_int.seq == sequence && mission_request_int.mission_type == type);
}

mavlink_message_t make_mission_count(unsigned count)
{
    mavlink_message_t message;
    mavlink_msg_mission_count_pack(
        own_address.system_id,
        own_address.component_id,
        &message,
        target_address.system_id,
        target_address.component_id,
        count,
        MAV_MISSION_TYPE_MISSION);
    return message;
}

TEST_F(MavlinkMissionTransferTest, DownloadMissionSendsMissionRequests)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    mmt.download_items_async(
        MAV_MISSION_TYPE_MISSION, [](Result result, const std::vector<ItemInt>& items) {
            UNUSED(items);
            UNUSED(result);
            EXPECT_TRUE(false);
        });
    mmt.do_work();

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_request_int(MAV_MISSION_TYPE_MISSION, 0, message);
                })));

    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));

    message_handler.process_message(make_mission_count(items.size()));
}

TEST_F(MavlinkMissionTransferTest, ReceiveIncomingMissionSendsMissionRequests)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));

    mmt.receive_incoming_items_async(
        MAV_MISSION_TYPE_MISSION,
        items.size(),
        target_address.component_id,
        [](Result result, const std::vector<ItemInt>& output_items) {
            UNUSED(output_items);
            UNUSED(result);
            EXPECT_TRUE(false);
        });

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_autopilot_mission_request_int(
                        MAV_MISSION_TYPE_MISSION, 0, target_address.component_id, message);
                })));

    mmt.do_work();

    message_handler.process_message(make_mission_count(items.size()));
}

TEST_F(MavlinkMissionTransferTest, DownloadMissionResendsMissionRequestsAndTimesOutEventually)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::promise<void> prom;
    auto fut = prom.get_future();
    mmt.download_items_async(
        MAV_MISSION_TYPE_MISSION, [&prom](Result result, const std::vector<ItemInt>& items) {
            UNUSED(items);
            EXPECT_EQ(result, Result::Timeout);
            prom.set_value();
        });
    mmt.do_work();

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_request_int(MAV_MISSION_TYPE_MISSION, 0, message);
                })))
        .Times(MavlinkMissionTransfer::retries);

    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));

    message_handler.process_message(make_mission_count(items.size()));

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(0)), std::future_status::timeout);

    // After the specified retries we should give up with a timeout.
    for (unsigned i = 0; i < MavlinkMissionTransfer::retries; ++i) {
        time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
        timeout_handler.run_once();
    }

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(
    MavlinkMissionTransferTest, ReceiveIncomingMissionResendsMissionRequestsAndTimesOutEventually)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));

    std::promise<void> prom;
    auto fut = prom.get_future();
    mmt.receive_incoming_items_async(
        MAV_MISSION_TYPE_MISSION,
        items.size(),
        target_address.component_id,
        [&prom](Result result, const std::vector<ItemInt>& output_items) {
            UNUSED(output_items);
            EXPECT_EQ(result, Result::Timeout);
            prom.set_value();
        });

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_autopilot_mission_request_int(
                        MAV_MISSION_TYPE_MISSION, 0, target_address.component_id, message);
                })))
        .Times(MavlinkMissionTransfer::retries);

    mmt.do_work();

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(0)), std::future_status::timeout);

    // After the specified retries we should give up with a timeout.
    for (unsigned i = 0; i < MavlinkMissionTransfer::retries; ++i) {
        time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
        timeout_handler.run_once();
    }

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

mavlink_message_t make_mission_item(const std::vector<ItemInt>& item_ints, std::size_t index)
{
    mavlink_message_t message;
    mavlink_msg_mission_item_int_pack(
        own_address.system_id,
        own_address.component_id,
        &message,
        target_address.system_id,
        target_address.component_id,
        index,
        item_ints[index].frame,
        item_ints[index].command,
        item_ints[index].current,
        item_ints[index].autocontinue,
        item_ints[index].param1,
        item_ints[index].param2,
        item_ints[index].param3,
        item_ints[index].param4,
        item_ints[index].x,
        item_ints[index].y,
        item_ints[index].z,
        item_ints[index].mission_type);
    return message;
}

TEST_F(MavlinkMissionTransferTest, DownloadMissionSendsAllMissionRequestsAndAck)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::vector<ItemInt> real_items;
    real_items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    real_items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));
    real_items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 2));

    std::promise<void> prom;
    auto fut = prom.get_future();
    mmt.download_items_async(
        MAV_MISSION_TYPE_MISSION,
        [&prom, &real_items](Result result, const std::vector<ItemInt>& items) {
            EXPECT_EQ(result, Result::Success);
            EXPECT_EQ(items, real_items);
            prom.set_value();
        });
    mmt.do_work();

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_request_int(MAV_MISSION_TYPE_MISSION, 0, message);
                })));

    message_handler.process_message(make_mission_count(real_items.size()));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_request_int(MAV_MISSION_TYPE_MISSION, 1, message);
                })));

    message_handler.process_message(make_mission_item(real_items, 0));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_request_int(MAV_MISSION_TYPE_MISSION, 2, message);
                })));

    message_handler.process_message(make_mission_item(real_items, 1));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_ack(
                        MAV_MISSION_TYPE_MISSION, MAV_MISSION_ACCEPTED, message);
                })));

    message_handler.process_message(make_mission_item(real_items, 2));

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, ReceiveIncomingMissionSendsAllMissionRequestsAndAck)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::vector<ItemInt> real_items;
    real_items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    real_items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));
    real_items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 2));

    std::promise<void> prom;
    auto fut = prom.get_future();
    mmt.receive_incoming_items_async(
        MAV_MISSION_TYPE_MISSION,
        real_items.size(),
        target_address.component_id,
        [&prom, &real_items](Result result, const std::vector<ItemInt>& items) {
            EXPECT_EQ(result, Result::Success);
            EXPECT_EQ(items, real_items);
            prom.set_value();
        });

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_autopilot_mission_request_int(
                        MAV_MISSION_TYPE_MISSION, 0, target_address.component_id, message);
                })));

    mmt.do_work();

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_autopilot_mission_request_int(
                        MAV_MISSION_TYPE_MISSION, 1, target_address.component_id, message);
                })));

    message_handler.process_message(make_mission_item(real_items, 0));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_autopilot_mission_request_int(
                        MAV_MISSION_TYPE_MISSION, 2, target_address.component_id, message);
                })));

    message_handler.process_message(make_mission_item(real_items, 1));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_autopilot_mission_ack(
                        MAV_MISSION_TYPE_MISSION,
                        MAV_MISSION_ACCEPTED,
                        target_address.component_id,
                        message);
                })));

    message_handler.process_message(make_mission_item(real_items, 2));

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, DownloadMissionResendsRequestItemAgainForSecondItem)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::promise<void> prom;
    auto fut = prom.get_future();
    mmt.download_items_async(
        MAV_MISSION_TYPE_MISSION, [&prom](Result result, const std::vector<ItemInt>& items) {
            UNUSED(items);
            EXPECT_EQ(result, Result::Timeout);
            prom.set_value();
        });
    mmt.do_work();

    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 2));

    // We almost exhaust the retries of the first one.
    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_request_int(MAV_MISSION_TYPE_MISSION, 0, message);
                })))
        .Times(MavlinkMissionTransfer::retries - 1);

    message_handler.process_message(make_mission_count(items.size()));

    for (unsigned i = 0; i < MavlinkMissionTransfer::retries - 2; ++i) {
        time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
        timeout_handler.run_once();
    }

    // This time we go over the retry limit.
    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_request_int(MAV_MISSION_TYPE_MISSION, 1, message);
                })))
        .Times(MavlinkMissionTransfer::retries);

    message_handler.process_message(make_mission_item(items, 0));

    for (unsigned i = 0; i < MavlinkMissionTransfer::retries; ++i) {
        time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
        timeout_handler.run_once();
    }

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, DownloadMissionDoesntHaveDuplicates)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::vector<ItemInt> real_items;
    real_items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    real_items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));
    real_items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 2));

    std::promise<void> prom;
    auto fut = prom.get_future();
    mmt.download_items_async(
        MAV_MISSION_TYPE_MISSION,
        [&prom, &real_items](Result result, const std::vector<ItemInt>& items) {
            EXPECT_EQ(result, Result::Success);
            EXPECT_EQ(items, real_items);
            prom.set_value();
        });
    mmt.do_work();

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_request_int(MAV_MISSION_TYPE_MISSION, 0, message);
                })));

    message_handler.process_message(make_mission_count(real_items.size()));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_request_int(MAV_MISSION_TYPE_MISSION, 1, message);
                })));

    message_handler.process_message(make_mission_item(real_items, 0));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_request_int(MAV_MISSION_TYPE_MISSION, 2, message);
                })));

    // Send a message 3 times, it should just get ignored.
    message_handler.process_message(make_mission_item(real_items, 1));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_ack(
                        MAV_MISSION_TYPE_MISSION, MAV_MISSION_ACCEPTED, message);
                })));

    message_handler.process_message(make_mission_item(real_items, 1));
    message_handler.process_message(make_mission_item(real_items, 1));

    message_handler.process_message(make_mission_item(real_items, 2));

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, ReceiveIncomingMissionResendsRequestItemAgainForSecondItem)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 2));

    std::promise<void> prom;
    auto fut = prom.get_future();
    mmt.receive_incoming_items_async(
        MAV_MISSION_TYPE_MISSION,
        items.size(),
        target_address.component_id,
        [&prom](Result result, const std::vector<ItemInt>& output_items) {
            UNUSED(output_items);
            EXPECT_EQ(result, Result::Timeout);
            prom.set_value();
        });

    // We almost exhaust the retries of the first one.
    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_autopilot_mission_request_int(
                        MAV_MISSION_TYPE_MISSION, 0, target_address.component_id, message);
                })))
        .Times(MavlinkMissionTransfer::retries - 1);

    mmt.do_work();

    for (unsigned i = 0; i < MavlinkMissionTransfer::retries - 2; ++i) {
        time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
        timeout_handler.run_once();
    }

    // This time we go over the retry limit.
    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_autopilot_mission_request_int(
                        MAV_MISSION_TYPE_MISSION, 1, target_address.component_id, message);
                })))
        .Times(MavlinkMissionTransfer::retries);

    message_handler.process_message(make_mission_item(items, 0));

    for (unsigned i = 0; i < MavlinkMissionTransfer::retries; ++i) {
        time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
        timeout_handler.run_once();
    }

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, DownloadMissionEmptyList)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::promise<void> prom;
    auto fut = prom.get_future();
    mmt.download_items_async(
        MAV_MISSION_TYPE_MISSION, [&prom](Result result, const std::vector<ItemInt>& items) {
            UNUSED(items);
            EXPECT_EQ(result, Result::Success);
            prom.set_value();
        });
    mmt.do_work();

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_ack(
                        MAV_MISSION_TYPE_MISSION, MAV_MISSION_ACCEPTED, message);
                })));

    message_handler.process_message(make_mission_count(0));

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    // We want to be sure a timeout is not still triggered later.
    time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
    timeout_handler.run_once();

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, ReceiveIncomingMissionEmptyList)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::promise<void> prom;
    auto fut = prom.get_future();
    mmt.receive_incoming_items_async(
        MAV_MISSION_TYPE_MISSION,
        0,
        target_address.component_id,
        [&prom](Result result, const std::vector<ItemInt>& items) {
            UNUSED(items);
            EXPECT_EQ(result, Result::Success);
            prom.set_value();
        });

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_autopilot_mission_ack(
                        MAV_MISSION_TYPE_MISSION,
                        MAV_MISSION_ACCEPTED,
                        target_address.component_id,
                        message);
                })));

    mmt.do_work();

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    // We want to be sure a timeout is not still triggered later.
    time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
    timeout_handler.run_once();

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, DownloadMissionTimeoutNotTriggeredDuringTransfer)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::vector<ItemInt> real_items;
    real_items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    real_items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));
    real_items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 2));

    std::promise<void> prom;
    auto fut = prom.get_future();
    mmt.download_items_async(
        MAV_MISSION_TYPE_MISSION,
        [&real_items, &prom](Result result, const std::vector<ItemInt>& items) {
            EXPECT_EQ(result, Result::Success);
            EXPECT_EQ(real_items, items);
            prom.set_value();
        });
    mmt.do_work();

    // We almost use up the max timeout in each cycle.
    time.sleep_for(std::chrono::milliseconds(
        static_cast<int>(timeout_s * MavlinkMissionTransfer::retries * 0.8 * 1000.)));
    timeout_handler.run_once();

    message_handler.process_message(make_mission_count(real_items.size()));

    time.sleep_for(std::chrono::milliseconds(
        static_cast<int>(timeout_s * MavlinkMissionTransfer::retries * 0.8 * 1000.)));
    timeout_handler.run_once();

    message_handler.process_message(make_mission_item(real_items, 0));

    time.sleep_for(std::chrono::milliseconds(
        static_cast<int>(timeout_s * MavlinkMissionTransfer::retries * 0.8 * 1000.)));
    timeout_handler.run_once();

    message_handler.process_message(make_mission_item(real_items, 1));

    time.sleep_for(std::chrono::milliseconds(
        static_cast<int>(timeout_s * MavlinkMissionTransfer::retries * 0.8 * 1000.)));
    timeout_handler.run_once();

    message_handler.process_message(make_mission_item(real_items, 2));

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, DownloadMissionCanBeCancelled)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::promise<void> prom;
    auto fut = prom.get_future();
    auto transfer = mmt.download_items_async(
        MAV_MISSION_TYPE_MISSION, [&prom](Result result, const std::vector<ItemInt>& items) {
            UNUSED(items);
            EXPECT_EQ(result, Result::Cancelled);
            prom.set_value();
        });
    mmt.do_work();

    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 2));

    message_handler.process_message(make_mission_count(items.size()));
    message_handler.process_message(make_mission_item(items, 0));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_ack(
                        MAV_MISSION_TYPE_MISSION, MAV_MISSION_OPERATION_CANCELLED, message);
                })));

    auto ptr = transfer.lock();
    EXPECT_TRUE(ptr);
    if (ptr) {
        ptr->cancel();
    }

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, ReceiveIncomingMissionCanBeCancelled)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::vector<ItemInt> items;
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));
    items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 2));

    std::promise<void> prom;
    auto fut = prom.get_future();
    auto transfer = mmt.receive_incoming_items_async(
        MAV_MISSION_TYPE_MISSION,
        items.size(),
        target_address.component_id,
        [&prom](Result result, const std::vector<ItemInt>& output_items) {
            UNUSED(output_items);
            EXPECT_EQ(result, Result::Cancelled);
            prom.set_value();
        });
    mmt.do_work();

    message_handler.process_message(make_mission_item(items, 0));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_autopilot_mission_ack(
                        MAV_MISSION_TYPE_MISSION,
                        MAV_MISSION_OPERATION_CANCELLED,
                        target_address.component_id,
                        message);
                })));

    auto ptr = transfer.lock();
    EXPECT_TRUE(ptr);
    if (ptr) {
        ptr->cancel();
    }

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, DownloadMissionWithProgress)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::vector<ItemInt> real_items;
    real_items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 0));
    real_items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 1));
    real_items.push_back(make_item(MAV_MISSION_TYPE_MISSION, 2));

    std::promise<void> prom;
    auto fut = prom.get_future();

    size_t num_progress_received = 0;
    float last_progress_received = NAN;

    mmt.download_items_async(
        MAV_MISSION_TYPE_MISSION,
        [&prom, &real_items](Result result, const std::vector<ItemInt>& items) {
            EXPECT_EQ(result, Result::Success);
            EXPECT_EQ(items, real_items);
            prom.set_value();
        },
        [&num_progress_received, &last_progress_received](float progress) {
            if (num_progress_received == 0) {
                EXPECT_EQ(progress, 0.0f);
            } else {
                EXPECT_GT(progress, last_progress_received);
            }
            ++num_progress_received;
            last_progress_received = progress;
        });
    mmt.do_work();

    message_handler.process_message(make_mission_count(real_items.size()));
    message_handler.process_message(make_mission_item(real_items, 0));
    message_handler.process_message(make_mission_item(real_items, 1));
    message_handler.process_message(make_mission_item(real_items, 2));

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    EXPECT_EQ(last_progress_received, 1.0f);
    EXPECT_GE(num_progress_received, 3);
    EXPECT_LE(num_progress_received, 10);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

bool is_correct_mission_clear_all(uint8_t type, const mavlink_message_t& message)
{
    if (message.msgid != MAVLINK_MSG_ID_MISSION_CLEAR_ALL) {
        return false;
    }

    mavlink_mission_clear_all_t clear_all;
    mavlink_msg_mission_clear_all_decode(&message, &clear_all);
    return (
        message.sysid == own_address.system_id && message.compid == own_address.component_id &&
        clear_all.target_system == target_address.system_id &&
        clear_all.target_component == target_address.component_id &&
        clear_all.mission_type == type);
}

TEST_F(MavlinkMissionTransferTest, ClearMissionSendsClear)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_clear_all(MAV_MISSION_TYPE_MISSION, message);
                })));

    std::promise<void> prom;
    auto fut = prom.get_future();
    mmt.clear_items_async(MAV_MISSION_TYPE_MISSION, [&prom](Result result) {
        EXPECT_EQ(result, Result::Success);
        prom.set_value();
    });
    mmt.do_work();

    message_handler.process_message(
        make_mission_ack(MAV_MISSION_TYPE_MISSION, MAV_RESULT_ACCEPTED));

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

bool is_correct_mission_set_current(uint16_t seq, const mavlink_message_t& message)
{
    if (message.msgid != MAVLINK_MSG_ID_MISSION_SET_CURRENT) {
        return false;
    }

    mavlink_mission_set_current_t mission_set_current;
    mavlink_msg_mission_set_current_decode(&message, &mission_set_current);
    return (
        message.sysid == own_address.system_id && message.compid == own_address.component_id &&
        mission_set_current.target_system == target_address.system_id &&
        mission_set_current.target_component == target_address.component_id &&
        mission_set_current.seq == seq);
}

mavlink_message_t make_mission_current(uint16_t seq)
{
    mavlink_message_t message;
    mavlink_msg_mission_current_pack(
        own_address.system_id, own_address.component_id, &message, seq, 0, 0, 0);
    return message;
}

TEST_F(MavlinkMissionTransferTest, SetCurrentSendsSetCurrent)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_set_current(2, message);
                })));

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.set_current_item_async(2, [&prom](Result result) {
        EXPECT_EQ(result, Result::Success);
        prom.set_value();
    });
    mmt.do_work();

    message_handler.process_message(make_mission_current(2));

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, SetCurrentWithRetransmissionAndTimeout)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_set_current(2, message);
                })))
        .Times(MavlinkMissionTransfer::retries);

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.set_current_item_async(2, [&prom](Result result) {
        EXPECT_EQ(result, Result::Timeout);
        prom.set_value();
    });
    mmt.do_work();

    for (unsigned i = 0; i < MavlinkMissionTransfer::retries; ++i) {
        time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
        timeout_handler.run_once();
    }

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, SetCurrentWithRetransmissionAndSuccess)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_set_current(2, message);
                })))
        .Times(MavlinkMissionTransfer::retries - 1);

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.set_current_item_async(2, [&prom](Result result) {
        EXPECT_EQ(result, Result::Success);
        prom.set_value();
    });
    mmt.do_work();

    for (unsigned i = 0; i < MavlinkMissionTransfer::retries - 2; ++i) {
        time.sleep_for(std::chrono::milliseconds(static_cast<int>(timeout_s * 1.1 * 1000.)));
        timeout_handler.run_once();
    }

    message_handler.process_message(make_mission_current(2));
    mmt.do_work();

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, SetCurrentWithInvalidInput)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.set_current_item_async(-1, [&prom](Result result) {
        EXPECT_EQ(result, Result::CurrentInvalid);
        prom.set_value();
    });
    mmt.do_work();

    EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, SetCurrentWithRetransmissionWhenWrong)
{
    ON_CALL(mock_sender, send_message(_)).WillByDefault(Return(true));

    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_set_current(2, message);
                })));

    std::promise<void> prom;
    auto fut = prom.get_future();

    mmt.set_current_item_async(2, [&prom](Result result) {
        EXPECT_EQ(result, Result::Success);
        prom.set_value();
    });
    mmt.do_work();

    // Retransmit to get correct feedback.
    EXPECT_CALL(mock_sender, send_message(Truly([](const mavlink_message_t& message) {
                    return is_correct_mission_set_current(2, message);
                })));

    message_handler.process_message(make_mission_current(1));
    mmt.do_work();

    message_handler.process_message(make_mission_current(2));

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}

TEST_F(MavlinkMissionTransferTest, IntMessagesNotSupported)
{
    mmt.set_int_messages_supported(false);

    {
        std::vector<ItemInt> items;
        std::promise<void> prom;
        auto fut = prom.get_future();

        mmt.upload_items_async(MAV_MISSION_TYPE_MISSION, items, [&prom](Result result) {
            EXPECT_EQ(result, Result::IntMessagesNotSupported);
            ONCE_ONLY;
            prom.set_value();
        });
        mmt.do_work();

        EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);
    }

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());

    {
        std::promise<void> prom;
        auto fut = prom.get_future();

        mmt.download_items_async(
            MAV_MISSION_TYPE_MISSION, [&prom](Result result, const std::vector<ItemInt>& items) {
                EXPECT_EQ(result, Result::IntMessagesNotSupported);
                UNUSED(items);
                ONCE_ONLY;
                prom.set_value();
            });
        mmt.do_work();

        EXPECT_EQ(fut.wait_for(std::chrono::seconds(1)), std::future_status::ready);
    }

    mmt.do_work();
    EXPECT_TRUE(mmt.is_idle());
}
