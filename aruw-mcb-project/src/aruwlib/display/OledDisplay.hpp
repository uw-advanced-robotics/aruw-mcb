#ifndef OLED_DISPLAY_HPP_
#define OLED_DISPLAY_HPP_

#include <modm/math/filter/debounce.hpp>
#include <modm/ui/menu/view_stack.hpp>

#include "aruwlib/rm-dev-board-a/board.hpp"

#include "MainMenu.hpp"
#include "sh1106.hpp"

namespace aruwlib
{
class Drivers;
namespace display
{
class OledDisplay
{
public:
    explicit OledDisplay(Drivers *drivers);
    OledDisplay(const OledDisplay &) = delete;
    OledDisplay &operator=(const OledDisplay &) = delete;
    ~OledDisplay() = default;

    void initialize();

    void update();

private:
    static constexpr int BUTTON_DEBOUNCE_SAMPLES = 10;
    static constexpr int DEBOUNCE_ADC_PRESSED_RANGE = 100;
    static constexpr int MIN_ADC_BEFORE_BUTTON_IN_IDLE_STATE = 3500;
    static constexpr int DOWN_ADC_VAL = 3300;
    static constexpr int UP_ADC_VAL = 2500;
    static constexpr int LEFT_ADC_VAL = 900;
    static constexpr int RIGHT_ADC_VAL = 1700;
    static constexpr int OK_ADC_VAL = 0;

    void handleButtonStatus();
    bool buttonIsIdle;

    Sh1106<
#ifndef ENV_SIMULATOR
        Board::DisplaySpiMaster,
        Board::DisplayCommand,
        Board::DisplayReset,
#endif
        128,
        64,
        false>
        display;

    modm::ViewStack vs;

    MainMenu mainMenu;

    modm::filter::Debounce<int> downButtonPressed;
    modm::filter::Debounce<int> upButtonPressed;
    modm::filter::Debounce<int> leftButtonPressed;
    modm::filter::Debounce<int> rightButtonPressed;
    modm::filter::Debounce<int> okButtonPressed;
};  // class OledDisplay
}  // namespace display
}  // namespace aruwlib

#endif  // OLED_DISPLAY_HPP_
