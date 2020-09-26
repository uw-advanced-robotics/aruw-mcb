#ifndef OLED_DISPLAY_MOCK_HPP_
#define OLED_DISPLAY_MOCK_HPP_

#include <aruwlib/display/OledDisplay.hpp>

namespace aruwlib
{
namespace mock
{
class OledDisplayMock : public display::OledDisplay 
{
public:
    explicit OledDisplayMock(Drivers *drivers) : display::OledDisplay(drivers) {}
    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, update, (), (override));
}  // class OledDisplayMock
}  // name
}  // name

#endif  // OLED_DISPLAY_MOCK_HPP_
