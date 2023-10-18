/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTRY_STRATEGY_MESSAGE_TYPES_HPP_
#define SENTRY_STRATEGY_MESSAGE_TYPES_HPP_

#include <cinttypes>
namespace aruwsrc::communication::serial
{
static constexpr uint16_t SENTRY_REQUEST_ROBOT_ID = 0x200;

static constexpr uint16_t SENTRY_RESPONSE_MESSAGE_ID = 0x201;

/**
 * Request message friendly robots send to the enemy
 */
enum class SentryRequestMessageType : uint8_t
{
    NONE = 0,
    GO_TO_FRIENDLY_BASE,
    GO_TO_ENEMY_BASE,
    GO_TO_FRIENDLY_SUPPLIER_ZONE,
    GO_TO_ENEMY_SUPPLIER_ZONE,
    GO_TO_CENTER_POINT,
    HOLD_FIRE,
    TOGGLE_MOVEMENT,
    TOGGLE_BEYBLADE,
    NUM_MESSAGE_TYPES,
};

/**
 * Response message the sentry broadcasts to all friendly robots.
 */
enum class SentryResponseMessageType : uint16_t
{
    NONE = 0,
    GO_TO_FRIENDLY_BASE,
    GO_TO_ENEMY_BASE,
    GO_TO_FRIENDLY_SUPPLIER_ZONE,
    GO_TO_ENEMY_SUPPLIER_ZONE,
    GO_TO_CENTER_POINT,
    HOLD_FIRE,
    MOVEMENT_ENABLED,
    MOVEMENT_DISABLED,
    BEYBLADE_ENABLED,
    BEYBLADE_DISABLED,
    NUM_MESSAGE_TYPES,
};
}  // namespace aruwsrc::communication::serial

#endif  //  SENTRY_STRATEGY_MESSAGE_TYPES_HPP_
