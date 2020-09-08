#ifndef REMOTE_MOCK_HPP_
#define REMOTE_MOCK_HPP_

#include <aruwlib/communication/remote.hpp>
#include <gmock/gmock.h>

class RemoteMock : public aruwlib::Remote
{
public:
    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, read, (), (override));
    MOCK_METHOD(bool, isConnected, (), (const override));
    MOCK_METHOD(float, getChannel, (aruwlib::Remote::Channel ch), (const override));
    MOCK_METHOD(SwitchState, getSwitch, (aruwlib::Remote::Switch sw), (const override));
    MOCK_METHOD(int16_t, getMouseX, (), (const override));
    MOCK_METHOD(int16_t, getMouseY, (), (const override));
    MOCK_METHOD(int16_t, getMouseZ, (), (const override));
    MOCK_METHOD(bool, getMouseL, (), (const override));
    MOCK_METHOD(bool, getMouseR, (), (const override));
    MOCK_METHOD(bool, keyPressed, (aruwlib::Remote::Key key), (const override));
    MOCK_METHOD(int16_t, getWheel, (), (const override));
    MOCK_METHOD(uint32_t, getUpdateCounter, (), (const override));
};  // class RemoteMock

#endif  // REMOTE_MOCK_HPP_
