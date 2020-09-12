#include <aruwlib/communication/remote.hpp>
#include <aruwlib/control/RemoteMapState.hpp>
#include <gtest/gtest.h>

using aruwlib::Remote;
using aruwlib::control::RemoteMapState;

TEST(RemoteMapState, default_constructor_default_remote_state)
{
    RemoteMapState ms;

    EXPECT_EQ(0, ms.getKeys());
    EXPECT_EQ(0, ms.getNegKeys());
    EXPECT_EQ(false, ms.getNegKeysUsed());
    EXPECT_EQ(false, ms.getRMouseButton());
    EXPECT_EQ(false, ms.getLMouseButton());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getRSwitch());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getLSwitch());
}

TEST(RemoteMapState, single_switchstate_constructor_only_switchstate_initialized)
{
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);

    EXPECT_EQ(0, ms.getKeys());
    EXPECT_EQ(0, ms.getNegKeys());
    EXPECT_EQ(false, ms.getNegKeysUsed());
    EXPECT_EQ(false, ms.getRMouseButton());
    EXPECT_EQ(false, ms.getLMouseButton());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getRSwitch());
    EXPECT_EQ(Remote::SwitchState::DOWN, ms.getLSwitch());
}

TEST(RemoteMapState, two_switchstate_constructor_only_switchstates_initialized)
{
    RemoteMapState ms(Remote::SwitchState::DOWN, Remote::SwitchState::UP);

    EXPECT_EQ(0, ms.getKeys());
    EXPECT_EQ(0, ms.getNegKeys());
    EXPECT_EQ(false, ms.getNegKeysUsed());
    EXPECT_EQ(false, ms.getRMouseButton());
    EXPECT_EQ(false, ms.getLMouseButton());
    EXPECT_EQ(Remote::SwitchState::UP, ms.getRSwitch());
    EXPECT_EQ(Remote::SwitchState::DOWN, ms.getLSwitch());
}

TEST(RemoteMapState, keyset_constructor_only_keyset_initialized)
{
    RemoteMapState ms({Remote::Key::A, Remote::Key::B});

    EXPECT_EQ(
        (1 << static_cast<int>(Remote::Key::A)) | (1 << static_cast<int>(Remote::Key::B)),
        ms.getKeys());
    EXPECT_EQ(0, ms.getNegKeys());
    EXPECT_EQ(false, ms.getNegKeysUsed());
    EXPECT_EQ(false, ms.getRMouseButton());
    EXPECT_EQ(false, ms.getLMouseButton());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getRSwitch());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getLSwitch());
}

TEST(RemoteMapState, keyset_negkeyset_constructor_only_keyset_negkeyset_initialized)
{
    RemoteMapState ms({Remote::Key::A, Remote::Key::B}, {Remote::Key::C, Remote::Key::D});

    EXPECT_EQ(
        (1 << static_cast<int>(Remote::Key::A)) | (1 << static_cast<int>(Remote::Key::B)),
        ms.getKeys());
    EXPECT_EQ(
        (1 << static_cast<int>(Remote::Key::C)) | (1 << static_cast<int>(Remote::Key::D)),
        ms.getNegKeys());
    EXPECT_EQ(true, ms.getNegKeysUsed());
    EXPECT_EQ(false, ms.getRMouseButton());
    EXPECT_EQ(false, ms.getLMouseButton());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getRSwitch());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getLSwitch());
}

TEST(RemoteMapState, equal_keyset_negkeyset_in_constructor_fails)
{
    RemoteMapState ms({Remote::Key::W}, {Remote::Key::W});

    EXPECT_EQ(1, ms.getKeys());
    EXPECT_EQ(0, ms.getNegKeys());
    EXPECT_EQ(false, ms.getNegKeysUsed());
    EXPECT_EQ(false, ms.getRMouseButton());
    EXPECT_EQ(false, ms.getLMouseButton());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getRSwitch());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getLSwitch());
}

TEST(RemoteMapState, intersecting_keyset_negkeyset_in_constructor_fails)
{
    RemoteMapState ms({Remote::Key::A, Remote::Key::B}, {Remote::Key::B, Remote::Key::C});
    EXPECT_EQ(
        (1 << static_cast<int>(Remote::Key::A)) | (1 << static_cast<int>(Remote::Key::B)),
        ms.getKeys());

    EXPECT_EQ(0, ms.getNegKeys());
    EXPECT_EQ(false, ms.getNegKeysUsed());
    EXPECT_EQ(false, ms.getRMouseButton());
    EXPECT_EQ(false, ms.getLMouseButton());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getRSwitch());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getLSwitch());
}

TEST(RemoteMapState, initLSwitch_sets_left_SwitchState)
{
    RemoteMapState ms;
    ms.initLSwitch(Remote::SwitchState::DOWN);

    EXPECT_EQ(Remote::SwitchState::DOWN, ms.getLSwitch());
}

TEST(RemoteMapState, initRSwitch_sets_right_SwitchState)
{
    RemoteMapState ms;
    ms.initRSwitch(Remote::SwitchState::DOWN);

    EXPECT_EQ(Remote::SwitchState::DOWN, ms.getRSwitch());
}

TEST(RemoteMapState, initKeys_sets_keyset)
{
    RemoteMapState ms;
    ms.initKeys(42);

    EXPECT_EQ(42, ms.getKeys());
}

TEST(RemoteMapState, initNegkeys_sets_negkeyset)
{
    RemoteMapState ms;
    ms.initNegKeys(42);

    EXPECT_EQ(42, ms.getNegKeys());
    EXPECT_EQ(true, ms.getNegKeysUsed());
}

TEST(RemoteMapState, initLMouseButton_sets_lMouseButton)
{
    RemoteMapState ms;
    ms.initLMouseButton();

    EXPECT_EQ(true, ms.getLMouseButton());
}

TEST(RemoteMapState, initRMouseButton_sets_RMouseButton)
{
    RemoteMapState ms;
    ms.initRMouseButton();

    EXPECT_EQ(true, ms.getRMouseButton());
}

TEST(RemoteMapState, operator_equals_default_constructed_equal)
{
    RemoteMapState ms1;
    RemoteMapState ms2;

    EXPECT_EQ(ms1, ms2);
    EXPECT_FALSE(ms1 != ms2);
}

TEST(RemoteMapState, operator_equals_all_states_initialized_same_equal)
{
    RemoteMapState ms1;
    ms1.initKeys(1);
    ms1.initNegKeys(2);
    ms1.initLMouseButton();
    ms1.initRMouseButton();
    ms1.initLSwitch(Remote::SwitchState::DOWN);
    ms1.initRSwitch(Remote::SwitchState::MID);
    RemoteMapState ms2;
    ms2.initKeys(1);
    ms2.initNegKeys(2);
    ms2.initLMouseButton();
    ms2.initRMouseButton();
    ms2.initLSwitch(Remote::SwitchState::DOWN);
    ms2.initRSwitch(Remote::SwitchState::MID);  
  
    EXPECT_EQ(ms1, ms2);
    EXPECT_FALSE(ms1 != ms2);
}

TEST(RemoteMapState, operator_equals_all_states_initialized_not_same_not_equal)
{
    RemoteMapState ms1, ms2;
    ms1.initKeys(1);
    ms1.initNegKeys(2);
    ms1.initLMouseButton();
    ms1.initRMouseButton();
    ms1.initLSwitch(Remote::SwitchState::DOWN);
    ms1.initRSwitch(Remote::SwitchState::MID);
    ms2.initKeys(~1);
    ms2.initNegKeys(~2);
    ms2.initRMouseButton();
    ms2.initLSwitch(Remote::SwitchState::MID);
    ms2.initRSwitch(Remote::SwitchState::UP);   
 
    EXPECT_NE(ms1, ms2);
    EXPECT_TRUE(ms1 != ms2);
}

// stateSubsetOf(const RemoteMapState &other) const;

TEST(RemoteMapState, stateSubset_true_if_default_constructed)
{
    RemoteMapState ms1, ms2;

    EXPECT_TRUE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubset_true_simple_equal_case)
{
    RemoteMapState ms1, ms2;
    ms1.initKeys(42);
    ms2.initKeys(42);

    EXPECT_TRUE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubset_true_if_simple_not_equal_case)
{
    RemoteMapState ms1, ms2;
    ms1.initLMouseButton();
    ms1.initKeys(42);
    ms2.initKeys(42);

    // EXPECT_FALSE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubset_key_subset)
{
    RemoteMapState ms1, ms2;
    ms1.initKeys(0xffff);
    ms1.initKeys(0x1234);
}

TEST(RemoteMapState, stateSubset_ignores_neg_keys)
{
    RemoteMapState ms1, ms2;
    ms1.initKeys(1);
    ms2.initKeys(3);

    EXPECT_TRUE(ms1.stateSubsetOf(ms2));
    EXPECT_FALSE(ms2.stateSubsetOf(ms1));

    ms1.initNegKeys(4);

    EXPECT_TRUE(ms1.stateSubsetOf(ms2));
    EXPECT_FALSE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubset_using_mouse_buttons)
{
    RemoteMapState ms1, ms2;
    ms1.initLMouseButton();
    ms1.initRMouseButton();
    ms2.initLMouseButton();

    EXPECT_FALSE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubset_using_different_switches)
{
    RemoteMapState ms1, ms2;
    ms1.initLSwitch(Remote::SwitchState::DOWN);
    ms2.initLSwitch(Remote::SwitchState::MID);

    EXPECT_FALSE(ms1.stateSubsetOf(ms2));
    EXPECT_FALSE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubset_using_matching_switches)
{
    RemoteMapState ms1, ms2;
    ms1.initLSwitch(Remote::SwitchState::DOWN);
    ms2.initLSwitch(Remote::SwitchState::MID);

    EXPECT_FALSE(ms1.stateSubsetOf(ms2));
    EXPECT_FALSE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubset_using_single_unknown_switch)
{
    RemoteMapState ms1, ms2;
    ms1.initLSwitch(Remote::SwitchState::DOWN);

    EXPECT_FALSE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}
