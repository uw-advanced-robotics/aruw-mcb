#include "modm/ui/display/graphic_display.hpp"

namespace aruwlib
{

namespace errors
{
    class OledTestDisplay : public modm::GraphicDisplay
    {
     public:
        uint16_t getWidth() const override;

        uint16_t getHeight() const override;

        void clear() override;

        void update() override;
    }
}

}