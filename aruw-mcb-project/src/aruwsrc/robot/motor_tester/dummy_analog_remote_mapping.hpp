#include "tap/control/command_mapping/analog_remote_mapping.hpp"


class ConstAnalogRemoteMapping : public tap::control::AnalogRemoteMapping
{
public:
    ConstAnalogRemoteMapping(float val) : val(val) {}
    inline float getValue() const override final { return val; }
private:
    float val;
};