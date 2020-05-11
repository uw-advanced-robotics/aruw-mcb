#ifndef UNIQUE_PTR_UNIT_TEST_HPP_
#define UNIQUE_PTR_UNIT_TEST_HPP_

#include <modm/architecture/interface/assert.hpp>

#include "unique_ptr.hpp"

using namespace modm;
using namespace aruwlib::arch;

void UniquePtr_TestMain()
{
    modm_assert_debug(1 == 1, "arch", "unique ptr", "1 == 1");

    UniquePtr<int> ptr_test_null(nullptr);
    // normally don't do this but for testing we do
    int* ptr_ref = new int(4);
    UniquePtr<int> ptr_test(ptr_ref);

    modm_assert_debug(ptr_test_null == nullptr, "arch", "unique ptr", "test nullptr equality with null");
    modm_assert_debug(!(ptr_test_null != nullptr), "arch", "unique ptr", "test nullptr inequality with null");

    modm_assert_debug(ptr_test != nullptr, "arch", "unique ptr", "test ptr equality with null");
    modm_assert_debug(!(ptr_test == nullptr), "arch", "unique ptr", "test ptr inequality with null");

    modm_assert_debug(ptr_test_null != ptr_test, "arch", "unique ptr", "test nullptr equality with other");
    modm_assert_debug(!(ptr_test_null == ptr_test), "arch", "unique ptr", "test nullptr inequality other");

    modm_assert_debug(ptr_test.get() == ptr_ref, "arch", "unique ptr", "test ptr equality");

    modm_assert_debug(*ptr_test == 4, "arch", "unique ptr", "test ptr dereference equality");
    int* new_ptr_ref = ptr_test.release();
    modm_assert_debug(new_ptr_ref == ptr_ref, "arch", "unique ptr", "test release, check pointer");

    modm_assert_debug(*new_ptr_ref == *ptr_ref, "arch", "unique ptr", "test release, check dereferenced pointer");

    UniquePtr<int> ptr_ref1(new int(5));
    UniquePtr<int> ptr_ref2(new int(6));

    ptr_ref1.swap(&ptr_ref2);

    modm_assert_debug(*ptr_ref1 == 6, "arch", "unique ptr", "test release, check ptr1 after swap");
    modm_assert_debug(*ptr_ref2 == 5, "arch", "unique ptr", "test release, check ptr2 after swap");

    // move ptr_ref2 to ptr_ref1
    ptr_ref1 = std::move(ptr_ref2);

    modm_assert_debug(*ptr_ref1 == 5, "arch", "unique ptr", "test move assignment");

    // move ptr_ref1 to a new ptr_ref3
    UniquePtr<int> ptr_ref3(std::move(ptr_ref1));

    modm_assert_debug(*ptr_ref3 == 5, "arch", "unique ptr", "test move constructor");

    delete ptr_ref;
}

#endif  // UNIQUE_PTR_UNIT_TEST_HPP_
