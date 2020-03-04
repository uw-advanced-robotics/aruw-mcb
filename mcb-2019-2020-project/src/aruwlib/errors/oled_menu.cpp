#include "oled_menu.hpp"
#include "src/aruwlib/display/sh1106.hpp"

namespace aruwlib
{

namespace errors
{

modm::MenuButtons::Button getButtonStatus();

OledMenu::OledMenu(modm::ViewStack *vs) :
AbstractMenu(vs, 1),
buttonIsIdle(true),
display_update_time(500),
timer(display_update_time),
viewStack(vs) {
    GpioOutputA6::setAnalogInput();
    Adc1::setPinChannel<GpioOutputA6>();
}

void OledMenu::addEntry(std::string name, std::string value) {
    ErrorMenuEntry newEntry{name, value};
    this->errorList.push_back(newEntry);
}

void OledMenu::update() {
    if (this->hasChanged()) {
        this->draw();
        this->handleButtonStatus();
    }
}

void shortButtonPress(modm::MenuButtons::Button button) {
    switch(button) {
        
    }
}

bool OledMenu::hasChanged() {
	if (timer.execute() || !buttonIsIdle)
	{
		if (!buttonIsIdle)
            buttonIsIdle = true;

		return true;
	}
	else
	{
		return false;
	}
}

void OledMenu::draw() {
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

uint16_t buttonADC;
void OledMenu::handleButtonStatus() {
    buttonADC = Adc1::readChannel(Adc1::getPinChannel<GpioOutputA6>());
    if (buttonADC > 3500)
    {
        buttonIsIdle = true;
        return;
    }
    if (buttonIsIdle)
    {
        buttonIsIdle = false;
        if (buttonADC > 2600) {
            shortButtonPress(modm::MenuButtons::DOWN);
        }
        else if (buttonADC > 2000)
        {
            shortButtonPress(modm::MenuButtons::UP);
        }
        else if (buttonADC > 1000)
        {
            shortButtonPress(modm::MenuButtons::RIGHT);
        }
        else if (buttonADC > 500)
        {
            shortButtonPress(modm::MenuButtons::LEFT);
        }
        else
        {
            shortButtonPress(modm::MenuButtons::OK);
        }
    }
}

}  // namespace errors

}  // namespace aruwlib
