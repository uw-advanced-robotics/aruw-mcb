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

#include "main_menu.hpp"

#include "tap/drivers.hpp"

using namespace tap::display;

namespace aruwsrc
{
namespace display
{
MainMenu::MainMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView>>* stack,
    tap::Drivers* drivers,
    serial::VisionCoprocessor* visionCoprocessor,
    can::TurretMCBCanComm* turretMCBCanCommBus1,
    can::TurretMCBCanComm* turretMCBCanCommBus2,
    communication::can::capbank::CapacitorBank* capacitorBank)
    : modm::StandardMenu<tap::display::DummyAllocator<modm::IAbstractView>>(stack, MAIN_MENU_ID),
      drivers(drivers),
      imuCalibrateMenu(stack, drivers),
      cvMenu(stack, drivers, visionCoprocessor),
      errorMenu(stack),
      hardwareTestMenu(stack, drivers),
      motorMenu(stack, drivers),
      commandSchedulerMenu(stack, drivers),
      refSerialMenu(stack, drivers),
      imuMenu(stack, &drivers->mpu6500),
      turretStatusMenuBus1(stack, turretMCBCanCommBus1),
      turretStatusMenuBus2(stack, turretMCBCanCommBus2),
      aboutMenu(stack),
      sentryStrategyMenu(stack, visionCoprocessor),
      capBankMenu(stack, capacitorBank),
      visionCoprocessor(visionCoprocessor),
      turretMCBCanCommBus1(turretMCBCanCommBus1),
      turretMCBCanCommBus2(turretMCBCanCommBus2),
      capacitorBank(capacitorBank)
{
}

void MainMenu::initialize()
{
    addEntry(
        ImuCalibrateMenu::getMenuName(),
        modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView>>(
            this,
            &MainMenu::addImuCalibrateMenuCallback));
    if (this->visionCoprocessor != nullptr)
        addEntry(
            CVMenu::getMenuName(),
            modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView>>(
                this,
                &MainMenu::addCVMenuCallback));
    addEntry(
        MotorMenu::getMenuName(),
        modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView>>(
            this,
            &MainMenu::addMotorMenuCallback));
    addEntry(
        RefSerialMenu::getMenuName(),
        modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView>>(
            this,
            &MainMenu::addRefSerialMenuCallback));
    addEntry(
        imuMenu.getMenuName(),
        modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView>>(
            this,
            &MainMenu::addImuMenuCallback));
    if (this->turretMCBCanCommBus1 != nullptr)
        addEntry(
            "Turret MCB Menu Bus 1",
            modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView>>(
                this,
                &MainMenu::addTurretMCBMenuBus1Callback));
    if (this->turretMCBCanCommBus2 != nullptr)
        addEntry(
            "Turret MCB Menu Bus 2",
            modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView>>(
                this,
                &MainMenu::addTurretMCBMenuBus2Callback));
    addEntry(
        CommandSchedulerMenu::getMenuName(),
        modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView>>(
            this,
            &MainMenu::addCommandSchedulerCallback));
    addEntry(
        HardwareTestMenu::getMenuName(),
        modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView>>(
            this,
            &MainMenu::addHardwareTestMenuCallback));
    addEntry(
        AboutMenu::getMenuName(),
        modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView>>(
            this,
            &MainMenu::addAboutMenuCallback));
    addEntry(
        SentryStrategyMenu::getMenuName(),
        modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView>>(
            this,
            &MainMenu::addSentryStrategyMenuCallback));
    
    if (this->capacitorBank != nullptr)
        addEntry(
            CapacitorBankMenu::getMenuName(),
            modm::MenuEntryCallback<DummyAllocator<modm::IAbstractView>>(
                this,
                &MainMenu::addCapacitorBankMenuCallback));

    setTitle("Main Menu");
}

void MainMenu::addImuCalibrateMenuCallback()
{
    ImuCalibrateMenu* icm = new (&imuCalibrateMenu) ImuCalibrateMenu(getViewStack(), drivers);
    getViewStack()->push(icm);
}

void MainMenu::addCVMenuCallback()
{
    CVMenu* cvm = new (&cvMenu) CVMenu(getViewStack(), drivers, visionCoprocessor);
    getViewStack()->push(cvm);
}

void MainMenu::addErrorMenuCallback()
{
    // em actually points to errorMenu
    ErrorMenu* em = new (&errorMenu) ErrorMenu(getViewStack());
    getViewStack()->push(em);
}

void MainMenu::addHardwareTestMenuCallback()
{
    HardwareTestMenu* htm = new (&hardwareTestMenu) HardwareTestMenu(getViewStack(), drivers);
    getViewStack()->push(htm);
}

void MainMenu::addMotorMenuCallback()
{
    MotorMenu* mm = new (&motorMenu) MotorMenu(getViewStack(), drivers);
    getViewStack()->push(mm);
}

void MainMenu::addPropertyTableCallback()
{
    // TODO, see issue #221
}

void MainMenu::addCommandSchedulerCallback()
{
    CommandSchedulerMenu* csm =
        new (&commandSchedulerMenu) CommandSchedulerMenu(getViewStack(), drivers);
    getViewStack()->push(csm);
}

void MainMenu::addRefSerialMenuCallback()
{
    RefSerialMenu* rsm = new (&refSerialMenu) RefSerialMenu(getViewStack(), drivers);
    getViewStack()->push(rsm);
}

void MainMenu::addImuMenuCallback()
{
    tap::communication::sensors::imu::ImuMenu* imc =
        new (&imuMenu) tap::communication::sensors::imu::ImuMenu(getViewStack(), &drivers->mpu6500);
    getViewStack()->push(imc);
}

void MainMenu::addTurretMCBMenuBus1Callback()
{
    TurretMCBMenu* tsm =
        new (&turretStatusMenuBus1) TurretMCBMenu(getViewStack(), turretMCBCanCommBus1);
    getViewStack()->push(tsm);
}

void MainMenu::addTurretMCBMenuBus2Callback()
{
    TurretMCBMenu* tsm =
        new (&turretStatusMenuBus2) TurretMCBMenu(getViewStack(), turretMCBCanCommBus2);
    getViewStack()->push(tsm);
}

void MainMenu::addAboutMenuCallback()
{
    AboutMenu* abtm = new (&aboutMenu) AboutMenu(getViewStack());
    getViewStack()->push(abtm);
}

void MainMenu::addSentryStrategyMenuCallback()
{
    SentryStrategyMenu* ssm =
        new (&sentryStrategyMenu) SentryStrategyMenu(getViewStack(), visionCoprocessor);
    getViewStack()->push(ssm);
}

void MainMenu::addCapacitorBankMenuCallback()
{
    CapacitorBankMenu* cbm = new (&capBankMenu) CapacitorBankMenu(getViewStack(), capacitorBank);
    getViewStack()->push(cbm);
}

}  // namespace display

}  // namespace aruwsrc
