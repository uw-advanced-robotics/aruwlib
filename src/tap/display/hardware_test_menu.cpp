/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "hardware_test_menu.hpp"

#include <algorithm>

#include "tap/control/command.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

namespace tap
{
namespace display
{
HardwareTestMenu::HardwareTestMenu(
    modm::ViewStack<DummyAllocator<modm::IAbstractView> >* vs,
    Drivers* drivers)
    : AbstractMenu<DummyAllocator<modm::IAbstractView> >(vs, HARDWARE_TEST_MENU_ID),
      drivers(drivers),
      vertScrollHandler(drivers, 0, MAX_ENTRIES_DISPLAYED)
{
}

void HardwareTestMenu::update() {}

void HardwareTestMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT)
    {
        drivers->commandScheduler.stopAllHardwareTests();
        this->remove();
    }
    else if (button == modm::MenuButtons::OK)
    {
        if (vertScrollHandler.getCursorIndex() == 0)
        {
            if (drivers->commandScheduler.countRunningHardwareTests() != 0)
            {
                drivers->commandScheduler.stopAllHardwareTests();
            }
            else
            {
                drivers->commandScheduler.runAllHardwareTests();
            }
            return;
        }

        int subsystemIndex = 1;
        for (auto it = drivers->commandScheduler.subMapBegin();
             it != drivers->commandScheduler.subMapEnd();
             it++)
        {
            if ((*it)->getTestCommand() != nullptr)
            {
                if (subsystemIndex++ == vertScrollHandler.getCursorIndex())
                {
                    if (drivers->commandScheduler.isRunningTest(*it))
                    {
                        drivers->commandScheduler.stopHardwareTest(*it);
                    }
                    else
                    {
                        drivers->commandScheduler.runHardwareTest(*it);
                    }
                    return;
                }
            }
        }
    }
    else
    {
        vertScrollHandler.onShortButtonPress(button);
    }
}

bool HardwareTestMenu::hasChanged()
{
    int runningTests = drivers->commandScheduler.countRunningHardwareTests();

    bool changed =
        vertScrollHandler.acknowledgeCursorChanged() || (runningTests != this->runningTests);
    this->runningTests = runningTests;
    return changed;
}

void HardwareTestMenu::draw()
{
    // Only do this once because it is assumed that all subsystems will be registered once
    // at the beginning of the program and subsystemListSize is not free
    if (vertScrollHandler.getSize() == 0)
    {
        int testableSubsystems = 0;
        for (auto it = drivers->commandScheduler.subMapBegin();
             it != drivers->commandScheduler.subMapEnd();
             it++)
        {
            if ((*it)->getTestCommand() != nullptr)
            {
                testableSubsystems++;
            }
        }
        vertScrollHandler.setSize(testableSubsystems + 1);
    }

    modm::GraphicDisplay& display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << HardwareTestMenu::getMenuName() << modm::endl;

    if (drivers->commandScheduler.countRunningHardwareTests() != 0)
    {
        display << "Running " << drivers->commandScheduler.countRunningHardwareTests() << " tests."
                << modm::endl;
        display << ((vertScrollHandler.getCursorIndex() == 0) ? ">" : " ");
        display << "[stop all] " << modm::endl;
    }
    else
    {
        display << "No tests running." << modm::endl;
        display << ((vertScrollHandler.getCursorIndex() == 0) ? ">" : " ");
        display << "[run all]" << modm::endl;
    }

    int subsystemIndex = 1;
    std::for_each(
        drivers->commandScheduler.subMapBegin(),
        drivers->commandScheduler.subMapEnd(),
        [&](control::Subsystem* sub) {
            if (sub->getTestCommand() != nullptr)
            {
                if (subsystemIndex <= (vertScrollHandler.getLargestIndexDisplayed()) &&
                    subsystemIndex >= (vertScrollHandler.getSmallestIndexDisplayed()))
                {
                    display << ((subsystemIndex == vertScrollHandler.getCursorIndex()) ? ">" : " ")
                            << ((drivers->commandScheduler.hasPassedTest(sub)) ? " + " : " x ")
                            << (drivers->commandScheduler.isRunningTest(sub) ? "[stop] "
                                                                             : "[run]  ")
                            << sub->getName() << modm::endl;
                }
                subsystemIndex++;
            }
        });
}
}  // namespace display
}  // namespace tap
