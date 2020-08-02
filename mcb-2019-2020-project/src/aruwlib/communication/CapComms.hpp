#ifndef CAP_COMMS_HPP_
#define CAP_COMMS_HPP_

namespace aruwlib
{
namespace communication
{
class CapComms
{
public:
    CapComms() = default;
    ~CapComms() = default;
    void initialize();
    void update();
};  // class CapComms
}  // namespace communication
}  // namespace aruwlib

#endif  // CAP_COMMS_HPP_
