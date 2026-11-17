/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "vision_coprocessor_menu.hpp"

#include "tap/drivers.hpp"

using namespace aruwsrc::communication::serial;

namespace aruwsrc::display
{
namespace
{
bool selectLine(int8_t &index, int8_t target)
{
    const bool selected = index == target;
    if (index < INT8_MAX)
    {
        index++;
    }
    return selected;
}

const char *fireRateName(VisionCoprocessor::FireRate rate)
{
    switch (rate)
    {
        case VisionCoprocessor::FireRate::ZERO:
            return "0";
        case VisionCoprocessor::FireRate::LOW:
            return "LOW";
        case VisionCoprocessor::FireRate::MEDIUM:
            return "MED";
        case VisionCoprocessor::FireRate::HIGH:
            return "HIGH";
        default:
            return "?";
    }
}

const char *plateIconName(VisionCoprocessor::PlateIcon icon)
{
    switch (icon)
    {
        case VisionCoprocessor::PlateIcon::UNKNOWN:
            return "UNK";
        case VisionCoprocessor::PlateIcon::HERO_ONE:
            return "HERO";
        case VisionCoprocessor::PlateIcon::ENGINEER_TWO:
            return "ENG";
        case VisionCoprocessor::PlateIcon::STANDARD_THREE:
            return "STD3";
        case VisionCoprocessor::PlateIcon::STANDARD_FOUR:
            return "STD4";
        case VisionCoprocessor::PlateIcon::STANDARD_FIVE:
            return "STD5";
        case VisionCoprocessor::PlateIcon::SEVEN:
            return "7";
        case VisionCoprocessor::PlateIcon::EIGHT:
            return "8";
        case VisionCoprocessor::PlateIcon::SENTRY:
            return "SENTRY";
        case VisionCoprocessor::PlateIcon::BASE:
            return "BASE";
        case VisionCoprocessor::PlateIcon::OUTPOST:
            return "OUTPOST";
        default:
            return "?";
    }
}

const char *motionStrategyName(uint8_t strategy)
{
    switch (static_cast<SentryMotionStrategyType>(strategy))
    {
        case SentryMotionStrategyType::IDLE:
            return "IDLE";
        case SentryMotionStrategyType::DEFAULT_STATE_MACHINE:
            return "DEFAULT";
        case SentryMotionStrategyType::PRACTICE_MATCH_STATE_MACHINE:
            return "PRACTICE";
        default:
            return "?";
    }
}
}  // namespace

VisionCoprocessorMenu::VisionCoprocessorMenu(
    modm::ViewStack<tap::display::DummyAllocator<modm::IAbstractView> > *vs,
    tap::Drivers *drivers,
    communication::serial::VisionCoprocessor *visionCoprocessor)
    : AbstractMenu<tap::display::DummyAllocator<modm::IAbstractView> >(
          vs,
          VISION_COPROCESSOR_MENU_ID),
      drivers(drivers),
      verticalScroll(drivers, 1, DISPLAY_MAX_ENTRIES),
      visionCoprocessor(visionCoprocessor)
{
}

void VisionCoprocessorMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    if (visionCoprocessor == nullptr)
    {
        display << "No vision coprocessor" << modm::endl;
        return;
    }

    for (int8_t i = verticalScroll.getSmallestIndexDisplayed();
         i <= verticalScroll.getLargestIndexDisplayed();
         i++)
    {
        display << (i == verticalScroll.getCursorIndex() ? "> " : "  ");
        emitLines(&display, i);
        display << modm::endl;
    }
}

void VisionCoprocessorMenu::update()
{
    if (visionCoprocessor == nullptr)
    {
        return;
    }

    const int8_t lineCount = emitLines(nullptr, -1);
    if (lineCount != verticalScroll.getSize())
    {
        verticalScroll.setSize(lineCount);
    }
}

void VisionCoprocessorMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT)
    {
        this->remove();
    }
    else
    {
        verticalScroll.onShortButtonPress(button);
    }
}

bool VisionCoprocessorMenu::hasChanged()
{
    return verticalScroll.acknowledgeCursorChanged() || updatePeriodicTimer.execute();
}

int8_t VisionCoprocessorMenu::emitLines(modm::IOStream *stream, int8_t target) const
{
    int8_t index = 0;
    const auto &odometry = visionCoprocessor->getLastOdometryData();
    const auto &chassis = odometry.chassisOdometry;

    if (selectLine(index, target) && stream != nullptr)
    {
        *stream << "CV online: " << (visionCoprocessor->isCvOnline() ? "true" : "false");
    }
    if (selectLine(index, target) && stream != nullptr)
    {
        stream->printf(
            "chassis pos: %.2f %.2f %.2f",
            static_cast<double>(chassis.xPos),
            static_cast<double>(chassis.yPos),
            static_cast<double>(chassis.zPos));
    }
    if (selectLine(index, target) && stream != nullptr)
    {
        stream->printf(
            "chassis rpy: %.2f %.2f %.2f",
            static_cast<double>(chassis.roll),
            static_cast<double>(chassis.pitch),
            static_cast<double>(chassis.yaw));
    }
    if (selectLine(index, target) && stream != nullptr)
    {
        *stream << "nTur: " << static_cast<int>(odometry.numTurrets);
    }
    for (uint8_t turret = 0; turret < control::turret::NUM_TURRETS; turret++)
    {
        const auto &pose = odometry.turretOdometry[turret];
        if (selectLine(index, target) && stream != nullptr)
        {
            stream->printf(
                "turret%u: %.2f %.2f %.2f",
                static_cast<unsigned>(turret),
                static_cast<double>(pose.xPos),
                static_cast<double>(pose.yPos),
                static_cast<double>(pose.zPos));
        }
        if (selectLine(index, target) && stream != nullptr)
        {
            stream->printf(
                "turret%u rpy: %.2f %.2f %.2f",
                static_cast<unsigned>(turret),
                static_cast<double>(pose.roll),
                static_cast<double>(pose.pitch),
                static_cast<double>(pose.yaw));
        }
    }

    if (selectLine(index, target) && stream != nullptr)
    {
        *stream << "TX id: " << static_cast<int>(visionCoprocessor->getLastRobotId());
    }
    if (selectLine(index, target) && stream != nullptr)
    {
        *stream << "strat: " << motionStrategyName(visionCoprocessor->getLastMotionStrategy());
    }
    if (selectLine(index, target) && stream != nullptr)
    {
        *stream << "bullets: " << visionCoprocessor->getLastBulletsRemaining();
    }

    for (uint8_t turret = 0; turret < control::turret::NUM_TURRETS; turret++)
    {
        const auto &aim = visionCoprocessor->getLastAimData(turret);
        const auto &state = aim.targetState;
        if (selectLine(index, target) && stream != nullptr)
        {
            *stream << "RX aim" << turret << " upd:" << static_cast<int>(state.updated)
                    << " t:" << aim.timestamp;
        }
        if (selectLine(index, target) && stream != nullptr)
        {
            *stream << "rate:" << fireRateName(state.firerate)
                    << " icon:" << plateIconName(state.icon);
        }
        if (selectLine(index, target) && stream != nullptr)
        {
            stream->printf(
                "p: %.2f %.2f %.2f",
                static_cast<double>(state.xPos),
                static_cast<double>(state.yPos),
                static_cast<double>(state.zPos));
        }
        if (selectLine(index, target) && stream != nullptr)
        {
            stream->printf(
                "v: %.2f %.2f %.2f",
                static_cast<double>(state.xVel),
                static_cast<double>(state.yVel),
                static_cast<double>(state.zVel));
        }
        if (selectLine(index, target) && stream != nullptr)
        {
            stream->printf(
                "a: %.2f %.2f %.2f",
                static_cast<double>(state.xAcc),
                static_cast<double>(state.yAcc),
                static_cast<double>(state.zAcc));
        }
        if (selectLine(index, target) && stream != nullptr)
        {
            stream->printf(
                "th: %.2f w: %.2f",
                static_cast<double>(state.theta),
                static_cast<double>(state.omega));
        }
        if (selectLine(index, target) && stream != nullptr)
        {
            stream->printf(
                "r: %.2f %.2f",
                static_cast<double>(state.radius0),
                static_cast<double>(state.radius1));
        }
        if (selectLine(index, target) && stream != nullptr)
        {
            stream->printf(
                "h: %.2f %.2f %.2f %.2f",
                static_cast<double>(state.plateHeights[0]),
                static_cast<double>(state.plateHeights[1]),
                static_cast<double>(state.plateHeights[2]),
                static_cast<double>(state.plateHeights[3]));
        }
    }

    const VisionCoprocessor::ArucoResetData *arucoSources[] = {
        &visionCoprocessor->getLastRealsenseArucoData(),
        &visionCoprocessor->getLastArducamArucoData(),
    };
    const char *arucoLabels[] = {"RX rs", "RX ac"};
    for (uint8_t source = 0; source < 2; source++)
    {
        const auto &aruco = *arucoSources[source];
        if (selectLine(index, target) && stream != nullptr)
        {
            *stream << arucoLabels[source] << " upd:" << static_cast<int>(aruco.updated)
                    << " tid:" << aruco.data.turretId;
        }
        if (selectLine(index, target) && stream != nullptr)
        {
            stream->printf(
                "p: %.2f %.2f %.2f",
                static_cast<double>(aruco.data.x),
                static_cast<double>(aruco.data.y),
                static_cast<double>(aruco.data.z));
        }
        if (selectLine(index, target) && stream != nullptr)
        {
            stream->printf(
                "q: %.2f %.2f %.2f %.2f",
                static_cast<double>(aruco.data.quatW),
                static_cast<double>(aruco.data.quatX),
                static_cast<double>(aruco.data.quatY),
                static_cast<double>(aruco.data.quatZ));
        }
        if (selectLine(index, target) && stream != nullptr)
        {
            stream->printf(
                "mag: %.2f ang: %.2f",
                static_cast<double>(aruco.data.cameraToTagMagnitude),
                static_cast<double>(aruco.data.cameraToTagAngle));
        }
        if (selectLine(index, target) && stream != nullptr)
        {
            *stream << "ts: " << static_cast<int64_t>(aruco.data.timestamp);
        }
    }

    const uint32_t setpointCount = visionCoprocessor->getLastAutoNavNumSetpoints();
    if (selectLine(index, target) && stream != nullptr)
    {
        stream->printf(
            "RX nav %lu spd:%.2f n:%lu",
            static_cast<unsigned long>(visionCoprocessor->getLastAutoNavSequenceNum()),
            static_cast<double>(visionCoprocessor->getLastAutoNavSpeed()),
            static_cast<unsigned long>(setpointCount));
    }
    for (uint32_t i = 0; i < setpointCount; i += 2)
    {
        if (selectLine(index, target) && stream != nullptr)
        {
            stream->printf(
                "%lu: %.2f %.2f",
                static_cast<unsigned long>(i),
                static_cast<double>(visionCoprocessor->getLastAutoNavSetpointX(i)),
                static_cast<double>(visionCoprocessor->getLastAutoNavSetpointY(i)));
            if (i + 1 < setpointCount)
            {
                stream->printf(
                    " %lu: %.2f %.2f",
                    static_cast<unsigned long>(i + 1),
                    static_cast<double>(visionCoprocessor->getLastAutoNavSetpointX(i + 1)),
                    static_cast<double>(visionCoprocessor->getLastAutoNavSetpointY(i + 1)));
            }
        }
    }

    const auto &orbits = visionCoprocessor->getLastRobotOrbitData();
    for (uint8_t i = 0; i < VisionCoprocessor::MAX_NUM_ROBOT_ORBITS; i++)
    {
        const auto &orbit = orbits.data[i];
        if (selectLine(index, target) && stream != nullptr)
        {
            stream->printf(
                "RX orb%u: %.2f %.2f %.2f",
                static_cast<unsigned>(i),
                static_cast<double>(orbit.x),
                static_cast<double>(orbit.y),
                static_cast<double>(orbit.z));
        }
        if (selectLine(index, target) && stream != nullptr)
        {
            stream->printf(
                "radius: %.2f type:%u",
                static_cast<double>(orbit.radius),
                static_cast<unsigned>(orbit.robotType));
        }
    }

    return index;
}
}  // namespace aruwsrc::display
