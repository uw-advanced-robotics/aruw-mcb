/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef MAIN_MENU_HPP_
#define MAIN_MENU_HPP_

#include "tap/communication/sensors/imu/imu_menu.hpp"
#include "tap/display/command_scheduler_menu.hpp"
#include "tap/display/dummy_allocator.hpp"
#include "tap/display/hardware_test_menu.hpp"
#include "tap/display/motor_menu.hpp"
#include "tap/display/ref_serial_menu.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "modm/ui/menu/standard_menu.hpp"

#include "about_menu.hpp"
#include "cv_menu.hpp"
#include "error_menu.hpp"
#include "imu_calibrate_menu.hpp"
#include "sentry_strategy_menu.hpp"
#include "turret_mcb_menu.hpp"
#include "mcb_lite_menu.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace display
{
class MainMenu : public modm::StandardMenu<tap::display::DummyAllocator<modm::IAbstractView>>
{
public:
    MainMenu(
        modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView>> *stack,
        tap::Drivers *drivers,
        serial::VisionCoprocessor *visionCoprocessor,
        can::TurretMCBCanComm *turretMCBCanCommBus1,
        can::TurretMCBCanComm *turretMCBCanCommBus2,
        aruwsrc::virtualMCB::MCBLite *mcbLite1,
        aruwsrc::virtualMCB::MCBLite *mcbLite2);

    virtual ~MainMenu() = default;

    /**
     * Adds entries to the menu to the necessary submenus.
     */
    void initialize();

private:
    static constexpr int MAIN_MENU_ID = 2;

    tap::Drivers *drivers;

    ImuCalibrateMenu imuCalibrateMenu;
    CVMenu cvMenu;
    ErrorMenu errorMenu;
    tap::display::HardwareTestMenu hardwareTestMenu;
    tap::display::MotorMenu motorMenu;
    tap::display::CommandSchedulerMenu commandSchedulerMenu;
    tap::display::RefSerialMenu refSerialMenu;
    tap::communication::sensors::imu::ImuMenu imuMenu;
    TurretMCBMenu turretStatusMenuBus1;
    TurretMCBMenu turretStatusMenuBus2;
    MCBLiteMenu mcbLiteMenu1;
    MCBLiteMenu mcbLiteMenu2;
    AboutMenu aboutMenu;
    SentryStrategyMenu sentryStrategyMenu;
    serial::VisionCoprocessor *visionCoprocessor;
    can::TurretMCBCanComm *turretMCBCanCommBus1;
    can::TurretMCBCanComm *turretMCBCanCommBus2;
    aruwsrc::virtualMCB::MCBLite *mcbLite1;
    aruwsrc::virtualMCB::MCBLite *mcbLite2;

    void addImuCalibrateMenuCallback();
    void addCVMenuCallback();
    void addErrorMenuCallback();
    void addHardwareTestMenuCallback();
    void addMotorMenuCallback();
    void addPropertyTableCallback();
    void addCommandSchedulerCallback();
    void addRefSerialMenuCallback();
    void addImuMenuCallback();
    void addTurretMCBMenuBus1Callback();
    void addTurretMCBMenuBus2Callback();
    void addAboutMenuCallback();
    void addSentryStrategyMenuCallback();
    void addMCBLiteMenu1Callback();
    void addMCBLiteMenu2Callback();
};  // class MainMenu
}  // namespace display
}  // namespace aruwsrc

#endif  // MAIN_MENU_HPP_
