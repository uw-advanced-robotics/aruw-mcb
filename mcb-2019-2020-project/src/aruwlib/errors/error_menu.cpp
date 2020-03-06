#include "error_menu.hpp"

namespace aruwlib
{

namespace errors
{

modm::MenuButtons::Button getButtonStatus();

ErrorMenu::ErrorMenu(modm::ViewStack *vs) :
AbstractMenu(vs, 1),
viewStack(vs),
display_update_time(500),
timer(display_update_time)
{
}

void ErrorMenu::addEntry(std::string name, std::string value) {
    ErrorMenuEntry newEntry{name, value};
    this->errorList.push_back(newEntry);
}

void ErrorMenu::update() {
    if (this->hasChanged()) {
        this->draw();
    }
}

void ErrorMenu::shortButtonPress(modm::MenuButtons::Button button) {
    switch(button) {
        case modm::MenuButtons::LEFT:
            break;
        case modm::MenuButtons::RIGHT:
            break;
        case modm::MenuButtons::DOWN:
            break;
        case modm::MenuButtons::UP:
            break;
        case modm::MenuButtons::OK:
            break;
    }
}

bool ErrorMenu::hasChanged() {
	return timer.execute();
}

void ErrorMenu::draw() {
    modm::GraphicDisplay* display = &getViewStack()->getDisplay();
	display->clear();
	display->setCursor(0,2);
	(*display) << this->title.c_str();
	display->drawLine(0, 10, display->getWidth(), 10);

	uint8_t count = this->errorList.size();
	auto iter = this->errorList.begin();

	for (int i = 0; i < this->maxEntries; i++)
	{
		if (i >= count)
			break;

		display->setCursor(4, 12+i*8);
		if (this->currentPosition == i) {
			(*display) << ">";
		}
		else {
			(*display) << " ";
		}

		(*display) << iter->name.c_str();
		display->setCursor(display->getWidth()- 8 - 5*6,12+i*8);
		(*display) << iter->value.c_str();
		++iter;
	}
}

}  // namespace errors

}  // namespace aruwlib
