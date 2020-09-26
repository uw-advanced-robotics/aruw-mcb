#include "MainMenu.hpp"

#include "ErrorMenu.hpp"

namespace aruwlib
{
namespace display
{
MainMenu::MainMenu(modm::ViewStack* stack, uint8_t identifier, Drivers* drivers)
    : modm::StandardMenu(stack, identifier),
      drivers(drivers)
{
}

void MainMenu::initialize()
{
    addEntry("Error Menu", modm::MenuEntryCallback(this, &MainMenu::addErrorMenuCallback));
    addEntry("Motor Menu", modm::MenuEntryCallback(this, &MainMenu::addMotorMenuCallback));
    addEntry(
        "Property Table Menu",
        modm::MenuEntryCallback(this, &MainMenu::addPropertyTableCallback));
    setTitle("Main Menu");
}

void MainMenu::addErrorMenuCallback()
{
    getViewStack()->push(new ErrorMenu(getViewStack(), drivers));
}

void MainMenu::addMotorMenuCallback() {}

void MainMenu::addPropertyTableCallback() {}
}  // namespace display

}  // namespace aruwlib
