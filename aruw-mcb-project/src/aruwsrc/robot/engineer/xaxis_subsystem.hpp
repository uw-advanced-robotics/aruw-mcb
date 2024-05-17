/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef XAXIS_SUBSYSTEM_HPP_
#define XAXIS_SUBSYSTEM_HPP_

#include "tap/communication/gpio/digital.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/util_macros.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc
{
namespace engineer
{
/**
 * This is a subsystem code for x-axis movement (moving the
 * grabber back and forward). Connect this to a digital output
 * pin. This controls a solenoid, which actuates a piston.
 */
class XAxisSubsystem : public tap::control::Subsystem
{
public:
    XAxisSubsystem(tap::Drivers *drivers, tap::gpio::Digital::OutputPin pin);

    mockable void setExtended(bool isExtended);

    mockable bool isExtended() const;

    void runHardwareTests() override;

    void onHardwareTestStart() override;

    void onHardwareTestComplete() override;

    const char *getName() const override { return "X-Axis"; }

private:
    tap::gpio::Digital::OutputPin pin;

    bool extended;

    uint64_t testTime;
};  // class XAxisSubsystem

}  // namespace engineer

}  // namespace aruwsrc

#endif  // XAXIS_SUBSYSTEM_HPP_
