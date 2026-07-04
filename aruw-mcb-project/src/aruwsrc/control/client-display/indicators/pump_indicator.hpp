#ifndef PUMP_INDICATOR_HPP_
#define PUMP_INDICATOR_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "aruwsrc/control/digital/digital_out_subsystem.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display::indicators
{
class PumpIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    PumpIndicator(
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        aruwsrc::control::digital::DigitalOutSubsystem &subsystem);

    void initialize() override final;

    modm::ResumableResult<void> update() override final;

private:
    aruwsrc::control::digital::DigitalOutSubsystem &subsystem;
    Tx::Graphic1Message pumpGraphic;

    static constexpr uint16_t PUMP_INDICATOR_RADIUS = 20;

    static constexpr uint16_t Y_POS = 865;
    static constexpr uint16_t X_POS = 123;
};

}  // namespace aruwsrc::control::client_display::indicators

#endif  // PUMP_INDICATOR_HPP_