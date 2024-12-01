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

#include <gtest/gtest.h>

#include "tap/display/vertical_scroll_logic_handler.hpp"
#include "tap/drivers.hpp"

using namespace tap::display;
using namespace tap;

static void expectScrollerConstraintsEq(
    const VerticalScrollLogicHandler &handler,
    int size,
    int largestIndex,
    int smallestIndex,
    int cursorIndex)
{
    EXPECT_EQ(size, handler.getSize());
    EXPECT_EQ(largestIndex, handler.getLargestIndexDisplayed());
    EXPECT_EQ(smallestIndex, handler.getSmallestIndexDisplayed());
    EXPECT_EQ(cursorIndex, handler.getCursorIndex());
}

TEST(VerticalScrollLogicHandler, constructor_negative_size_raises_error)
{
    const int8_t SIZE = -1;
    const int8_t ENTRIES = 5;
    Drivers drivers;
    VerticalScrollLogicHandler scroller(&drivers, SIZE, ENTRIES);

    expectScrollerConstraintsEq(scroller, SIZE, SIZE - 1, 0, 0);
    EXPECT_CALL(drivers.errorController, addToErrorList).Times(2);

    scroller.onShortButtonPress(modm::MenuButtons::DOWN);
    scroller.onShortButtonPress(modm::MenuButtons::UP);

    expectScrollerConstraintsEq(scroller, SIZE, SIZE - 1, 0, 0);
}

TEST(
    VerticalScrollLogicHandler,
    constructor_negative_max_entries_raises_error_and_sets_default_max_entries_to_1)
{
    const int8_t SIZE = 5;
    const int8_t ENTRIES = -1;
    Drivers drivers;
    EXPECT_CALL(drivers.errorController, addToErrorList);
    VerticalScrollLogicHandler scroller(&drivers, SIZE, ENTRIES);

    expectScrollerConstraintsEq(scroller, SIZE, 0, 0, 0);

    scroller.onShortButtonPress(modm::MenuButtons::DOWN);
    scroller.onShortButtonPress(modm::MenuButtons::UP);

    expectScrollerConstraintsEq(scroller, SIZE, 0, 0, 0);
}

TEST(
    VerticalScrollLogicHandler,
    constructor_0_max_entries_raises_error_and_sets_default_max_entries_to_1)
{
    const int8_t SIZE = 5;
    const int8_t ENTRIES = 0;
    Drivers drivers;
    EXPECT_CALL(drivers.errorController, addToErrorList);
    VerticalScrollLogicHandler scroller(&drivers, SIZE, ENTRIES);

    expectScrollerConstraintsEq(scroller, SIZE, 0, 0, 0);

    scroller.onShortButtonPress(modm::MenuButtons::DOWN);
    scroller.onShortButtonPress(modm::MenuButtons::UP);

    expectScrollerConstraintsEq(scroller, SIZE, 0, 0, 0);
}

TEST(
    VerticalScrollLogicHandler,
    constructor_0_size_entries_raises_error_and_sets_default_max_entries_to_1)
{
    const int8_t SIZE = 0;
    const int8_t ENTRIES = 5;
    Drivers drivers;
    VerticalScrollLogicHandler scroller(&drivers, SIZE, ENTRIES);

    expectScrollerConstraintsEq(scroller, SIZE, SIZE - 1, 0, 0);

    scroller.onShortButtonPress(modm::MenuButtons::DOWN);
    scroller.onShortButtonPress(modm::MenuButtons::UP);

    expectScrollerConstraintsEq(scroller, SIZE, SIZE - 1, 0, 0);
}

TEST(
    VerticalScrollLogicHandler,
    constructor_max_entries_greater_than_size_display_indices_bounded_by_size)
{
    const int8_t ENTRIES = 10;
    const int8_t SIZE = 5;
    Drivers drivers;
    VerticalScrollLogicHandler scroller(&drivers, SIZE, ENTRIES);
    expectScrollerConstraintsEq(scroller, SIZE, SIZE - 1, 0, 0);

    scroller.onShortButtonPress(modm::MenuButtons::UP);
    expectScrollerConstraintsEq(scroller, SIZE, SIZE - 1, 0, 0);

    for (int i = 0; i < SIZE; i++)
    {
        expectScrollerConstraintsEq(scroller, SIZE, SIZE - 1, 0, i);
        scroller.onShortButtonPress(modm::MenuButtons::DOWN);
    }
    scroller.onShortButtonPress(modm::MenuButtons::DOWN);
    for (int i = 0; i < SIZE; i++)
    {
        expectScrollerConstraintsEq(scroller, SIZE, SIZE - 1, 0, SIZE - 1 - i);
        scroller.onShortButtonPress(modm::MenuButtons::UP);
    }
    scroller.onShortButtonPress(modm::MenuButtons::UP);
    expectScrollerConstraintsEq(scroller, SIZE, SIZE - 1, 0, 0);
}

TEST(
    VerticalScrollLogicHandler,
    constructor_size_greater_than_max_entries_display_indices_bounded_by_max_entries)
{
    const int8_t ENTRIES = 5;
    const int8_t SIZE = 12;
    Drivers drivers;
    VerticalScrollLogicHandler scroller(&drivers, SIZE, ENTRIES);
    expectScrollerConstraintsEq(scroller, SIZE, ENTRIES - 1, 0, 0);

    scroller.onShortButtonPress(modm::MenuButtons::UP);
    expectScrollerConstraintsEq(scroller, SIZE, ENTRIES - 1, 0, 0);

    int i;
    for (i = 0; i < ENTRIES; i++)
    {
        expectScrollerConstraintsEq(scroller, SIZE, ENTRIES - 1, 0, i);
        scroller.onShortButtonPress(modm::MenuButtons::DOWN);
    }
    for (; i < SIZE; i++)
    {
        expectScrollerConstraintsEq(scroller, SIZE, i, i - ENTRIES + 1, i);
        scroller.onShortButtonPress(modm::MenuButtons::DOWN);
    }
    expectScrollerConstraintsEq(scroller, SIZE, SIZE - 1, SIZE - ENTRIES, SIZE - 1);

    for (i = SIZE - 1; i >= SIZE - ENTRIES; i--)
    {
        expectScrollerConstraintsEq(scroller, SIZE, SIZE - 1, SIZE - ENTRIES, i);
        scroller.onShortButtonPress(modm::MenuButtons::UP);
    }
    for (; i >= 0; i--)
    {
        expectScrollerConstraintsEq(scroller, SIZE, i + ENTRIES - 1, i, i);
        scroller.onShortButtonPress(modm::MenuButtons::UP);
    }
    expectScrollerConstraintsEq(scroller, SIZE, ENTRIES - 1, 0, 0);
}

TEST(VerticalScrollLogicHandler, acknowledgeCursorChanged_returns_false_when_cursor_not_changd)
{
    Drivers drivers;
    VerticalScrollLogicHandler scroller(&drivers, 10, 10);
    EXPECT_FALSE(scroller.acknowledgeCursorChanged());
    scroller.onShortButtonPress(modm::MenuButtons::UP);
    EXPECT_FALSE(scroller.acknowledgeCursorChanged());
}

TEST(
    VerticalScrollLogicHandler,
    acknowledgeCursorChanged_returns_true_when_cursor_moved_and_false_subsequently)
{
    Drivers drivers;
    VerticalScrollLogicHandler scroller(&drivers, 10, 10);
    scroller.onShortButtonPress(modm::MenuButtons::DOWN);
    EXPECT_TRUE(scroller.acknowledgeCursorChanged());
    EXPECT_FALSE(scroller.acknowledgeCursorChanged());
    EXPECT_FALSE(scroller.acknowledgeCursorChanged());
    scroller.onShortButtonPress(modm::MenuButtons::UP);
    EXPECT_TRUE(scroller.acknowledgeCursorChanged());
    EXPECT_FALSE(scroller.acknowledgeCursorChanged());
    EXPECT_FALSE(scroller.acknowledgeCursorChanged());
}

TEST(VerticalScrollLogicHandler, setSize_updates_size_cursor_display_indices_if_cursorIndex_ge_size)
{
    const int8_t ENTRIES = 10;
    const int8_t SIZE = 10;
    const int8_t NEW_SIZE = 5;
    Drivers drivers;
    VerticalScrollLogicHandler scroller(&drivers, SIZE, ENTRIES);
    expectScrollerConstraintsEq(scroller, SIZE, ENTRIES - 1, 0, 0);

    for (int i = 0; i < SIZE; i++)
    {
        scroller.onShortButtonPress(modm::MenuButtons::DOWN);
    }
    expectScrollerConstraintsEq(scroller, SIZE, ENTRIES - 1, 0, SIZE - 1);

    scroller.setSize(NEW_SIZE);
    expectScrollerConstraintsEq(scroller, NEW_SIZE, NEW_SIZE - 1, 0, NEW_SIZE - 1);
}

TEST(
    VerticalScrollLogicHandler,
    setSize_updates_size_cursor_display_indices_if_cursorIndex_ge_size_and_maxEntries_lt_size)
{
    const int8_t ENTRIES = 3;
    const int8_t SIZE = 10;
    const int8_t NEW_SIZE = 5;
    Drivers drivers;
    VerticalScrollLogicHandler scroller(&drivers, SIZE, ENTRIES);
    expectScrollerConstraintsEq(scroller, SIZE, ENTRIES - 1, 0, 0);

    for (int i = 0; i < SIZE; i++)
    {
        scroller.onShortButtonPress(modm::MenuButtons::DOWN);
    }
    expectScrollerConstraintsEq(scroller, SIZE, SIZE - 1, SIZE - ENTRIES, SIZE - 1);

    scroller.setSize(NEW_SIZE);
    expectScrollerConstraintsEq(scroller, NEW_SIZE, NEW_SIZE - 1, NEW_SIZE - ENTRIES, NEW_SIZE - 1);
}
TEST(VerticalScrollLogicHandler, setSize_negative_size_doesnt_update)
{
    const int8_t ENTRIES = 3;
    const int8_t SIZE = 10;

    Drivers drivers;
    VerticalScrollLogicHandler scroller(&drivers, SIZE, ENTRIES);
    expectScrollerConstraintsEq(scroller, SIZE, ENTRIES - 1, 0, 0);
    scroller.setSize(-1);
    expectScrollerConstraintsEq(scroller, SIZE, ENTRIES - 1, 0, 0);
}
