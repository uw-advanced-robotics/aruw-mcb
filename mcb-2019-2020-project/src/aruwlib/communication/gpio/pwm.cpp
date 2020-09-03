#include "pwm.hpp"

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/rm-dev-board-a/board.hpp"

using namespace Board;

namespace aruwlib
{
namespace gpio
{
void Pwm::init()
{
#ifndef ENV_SIMULATOR
    Timer8::connect<PWMOutPinW::Ch1, PWMOutPinX::Ch2, PWMOutPinY::Ch3, PWMOutPinZ::Ch4>();
    Timer8::enable();
    Timer8::setMode(Timer8::Mode::UpCounter);

    Timer8::setPrescaler(Board::SystemClock::APB2_PRESCALER);
    Timer8::setOverflow(Board::SystemClock::PWM_RESOLUTION);
    Timer3::enable();
    Timer3::setMode(Timer3::Mode::UpCounter);
    Timer3::setPrescaler(65535);
    Timer3::setPrescaler(Board::SystemClock::APB2_PRESCALER);
    Timer3::setOverflow(Board::SystemClock::PWM_RESOLUTION);
#endif

    Timer8::start();
    Timer8::enableOutput();
#endif
}

// void buzzer_note(uint16_t freq) {
//   // prescaler = (SYS CLOCK / (Clock Div * Period / 2)) / frequency
//   uint32_t prescaler = 11250000 / freq;
//   tim_buzzer_config(prescaler);
// }


void Pwm::writeAll(float duty)
{
#ifndef ENV_SIMULATOR
    write(duty, Pin::W);
    write(duty, Pin::X);
    write(duty, Pin::Y);
    write(duty, Pin::Z);
#endif
}

void Pwm::write(float duty, Pin pin)
{
#ifndef ENV_SIMULATOR
    // if (pin != BUZZER)
    // {
    //     duty = aruwlib::algorithms::limitVal<float>(duty, 0.0f, 1.0f);
    //     Timer8::configureOutputChannel(
    //         static_cast<int>(pin),
    //         Timer8::OutputCompareMode::Pwm,
    //         Board::SystemClock::PWM_RESOLUTION * duty);
    // }
    // else
    // {
    Timer3::configureOutputChannel(
        1, Timer3::OutputCompareMode::Pwm, duty);
    // }
#endif
}
}  // namespace gpio

}  // namespace aruwlib
