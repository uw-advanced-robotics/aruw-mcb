#ifndef DUMMY_ALLOCATOR_HPP_
#define DUMMY_ALLOCATOR_HPP_

#include <modm/utils/allocator/allocator_base.hpp>

namespace aruwlib
{
namespace display
{
template <typename T>
class DummyAllocator : public modm::allocator::AllocatorBase<T>
{
public:
    DummyAllocator() : modm::allocator::AllocatorBase<T>() {}

    DummyAllocator(const DummyAllocator& other) = default;

    T* allocate(size_t) {}

    void deallocate(T*) {}
};  // class DummyAllocator
}  // namespace display
}  // namespace aruwlib

#endif  // DUMMY_ALLOCATOR_HPP_
